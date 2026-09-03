package frc.robot.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.imu.Imu;
import frc.robot.localization.Localization;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.robot_manager.hopper_manager.HopperState;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.util.TiltCompensation;
import java.util.ArrayList;
import java.util.List;

/**
 * Headless end-to-end test of the "stuck on ball while scoring" feature. Only active in simulation
 * when the SIM_SELF_TEST environment variable is set. Selects the RIGHT auto, force-enables
 * autonomous via {@link DriverStationSim}, drives over the bump onto the virtual ball, and asserts
 * on the full detect -> compensate -> back off -> recover cycle. Prints exactly one SIM_SELF_TEST
 * PASS/FAIL line.
 */
public class StuckOnBallSimSelfTest {
  private static final double SETUP_SECONDS = 1.0;
  private static final double TIMEOUT_SECONDS = 45.0;

  private static String formatPose(Pose2d pose) {
    return "(%.2f, %.2f, %.1fdeg)"
        .formatted(pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }

  private final RobotManager robotManager;
  private final Localization localization;
  private final Imu imu;
  private final StuckOnBallSim stuckOnBallSim;
  private final HopperManager hopperManager;

  private final ShooterHood shooterHood;

  // Keep the publishers as fields so they aren't garbage collected
  private final StringPublisher autoSelectionPublisher =
      NetworkTableInstance.getDefault()
          .getStringTopic("/SmartDashboard/Autos/SelectedAuto/selected")
          .publish();

  // Nudge the ball 0.09 m before the shoot point: the auto settles ~0.19 m short of its target,
  // so with the ball exactly at the shoot point the entry tilt is only ~6.8 deg and the IMU
  // flat-offset filter absorbs it before the robot can back off a full 0.1 m. Landing ~0.1 m
  // deep on the ball (~8 deg entry tilt) exercises the full backoff the feature was tuned for.
  private final DoublePublisher ballXOffsetPublisher =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/Tunable/Sim/StuckOnBall/BallXOffset")
          .publish();
  private double startTime = -1.0;
  private double enableTime = -1.0;

  private boolean printed = false;

  private Pose2d enablePose = Pose2d.kZero;
  // Assertion first-satisfied timestamps (seconds since enable; -1 = not yet satisfied)
  private double robotMovedAt = -1.0;
  private double beachedAt = -1.0;
  private double hoodCompensatedAt = -1.0;
  private double backedOffAt = -1.0;
  private double recoveredToScoreAt = -1.0;

  // Informational only: the hopper sim doesn't always reach a scoring state reliably
  private double stillScoringAt = -1.0;
  // Diagnostics
  private double maxTiltSeenDegrees = 0.0;
  private double hoodOffsetMin = Double.POSITIVE_INFINITY;
  private double hoodOffsetMax = Double.NEGATIVE_INFINITY;
  private double beachEnterDistance = -1.0;
  private double beachExitDistance = -1.0;
  private double maxBeachedDistance = -1.0;
  private RobotState lastLoggedState = null;

  private final List<String> timeline = new ArrayList<>();

  public StuckOnBallSimSelfTest(
      RobotManager robotManager,
      Localization localization,
      Imu imu,
      StuckOnBallSim stuckOnBallSim,
      HopperManager hopperManager,
      ShooterHood shooterHood) {
    this.robotManager = robotManager;
    this.localization = localization;
    this.imu = imu;
    this.stuckOnBallSim = stuckOnBallSim;
    this.hopperManager = hopperManager;
    this.shooterHood = shooterHood;
  }

  public void periodic() {
    double now = Timer.getFPGATimestamp();
    if (startTime < 0.0) {
      startTime = now;
    }
    double elapsed = now - startTime;

    if (elapsed < SETUP_SECONDS) {
      // SETUP: stay disabled, pick red alliance + the RIGHT auto. Autos recreates the selected
      // auto and continuously resets the pose to the auto start while disabled.
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.setAutonomous(true);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      autoSelectionPublisher.set("RIGHT");
      ballXOffsetPublisher.set(-0.09);
      return;
    }

    // RUN: re-assert every loop so we win over the sim GUI DriverStation
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    if (enableTime < 0.0) {
      enableTime = now;
      enablePose = localization.getPose();
      System.out.println("SIM_SELF_TEST enabled at t=" + now + " pose=" + formatPose(enablePose));
    }
    double t = now - enableTime;

    if (printed) {
      return;
    }

    trackAssertions(t);

    if (allRequiredMet()) {
      printResult(true, t, "");
      printed = true;
    } else if (t > TIMEOUT_SECONDS) {
      printResult(false, t, unmetAssertions());
      printed = true;
    }
  }

  private boolean allRequiredMet() {
    return robotMovedAt >= 0.0
        && beachedAt >= 0.0
        && hoodCompensatedAt >= 0.0
        && backedOffAt >= 0.0
        && recoveredToScoreAt >= 0.0;
  }

  private void printResult(boolean pass, double t, String unmet) {
    String summary =
        ("maxTilt=%.2fdeg hoodOffset=[%.2f,%.2f]deg backoff=%.3fm (enter=%.3f exit=%.3f)"
                + " movedAt=%.2fs beachedAt=%.2fs hoodCompAt=%.2fs backedOffAt=%.2fs"
                + " recoveredAt=%.2fs stillScoringAt=%.2fs states=[%s]")
            .formatted(
                maxTiltSeenDegrees,
                hoodOffsetMin == Double.POSITIVE_INFINITY ? 0.0 : hoodOffsetMin,
                hoodOffsetMax == Double.NEGATIVE_INFINITY ? 0.0 : hoodOffsetMax,
                beachEnterDistance >= 0.0 && beachExitDistance >= 0.0
                    ? beachExitDistance - beachEnterDistance
                    : 0.0,
                beachEnterDistance,
                beachExitDistance,
                robotMovedAt,
                beachedAt,
                hoodCompensatedAt,
                backedOffAt,
                recoveredToScoreAt,
                stillScoringAt,
                String.join(" -> ", timeline));
    String line =
        pass
            ? "SIM_SELF_TEST PASS " + summary
            : "SIM_SELF_TEST FAIL: unmet=[" + unmet + "] " + summary;
    System.out.println(line);
    DriverStation.reportWarning(line, false);
  }

  private void trackAssertions(double t) {
    var state = robotManager.getState();

    if (state != lastLoggedState) {
      String entry = "%.2f:%s".formatted(t, state);
      timeline.add(entry);
      System.out.println("SIM_SELF_TEST timeline t=" + "%.2f".formatted(t) + " state=" + state);
      lastLoggedState = state;
    }

    double pitch = imu.getPitch();
    double roll = imu.getRoll();
    maxTiltSeenDegrees = Math.max(maxTiltSeenDegrees, Math.hypot(pitch, roll));

    // ROBOT_MOVED
    if (robotMovedAt < 0.0
        && localization.getPose().getTranslation().getDistance(enablePose.getTranslation()) > 1.0) {
      robotMovedAt = t;
    }

    boolean beached = state == RobotState.SCORE_STUCK_ON_BALL;

    // BEACHED
    if (beached && beachedAt < 0.0) {
      beachedAt = t;
      beachEnterDistance = stuckOnBallSim.getDistanceToBall();
    }

    if (beached) {
      // HOOD_COMPENSATED: offset is live, matches the compensation formula, and opposes pitch
      double offset = shooterHood.getScoreAngleOffsetDegrees();
      hoodOffsetMin = Math.min(hoodOffsetMin, offset);
      hoodOffsetMax = Math.max(hoodOffsetMax, offset);
      if (hoodCompensatedAt < 0.0) {
        double expected = TiltCompensation.getHoodCompensationDegrees(pitch, roll);
        if (!MathUtil.isNear(0, offset, 1.0)
            && MathUtil.isNear(offset, expected, 0.5)
            && offset * pitch < 0.0) {
          hoodCompensatedAt = t;
        }
      }

      // STILL_SCORING (informational)
      var hopperState = hopperManager.getState();
      if (stillScoringAt < 0.0
          && (hopperState == HopperState.SCORE || hopperState == HopperState.SCORE_AND_INTAKE)) {
        stillScoringAt = t;
      }
    }

    // Track the max distance-to-ball during the first beached episode. The state can flap
    // SCORE_STUCK_ON_BALL -> PREPARE_SCORE -> SCORE_STUCK_ON_BALL within a few loops (mechanism
    // atGoal conditions re-evaluate), so measuring the first exit edge undercounts the backoff;
    // the episode max captures the actual creep away from the ball.
    if (beachedAt >= 0.0 && recoveredToScoreAt < 0.0) {
      maxBeachedDistance = Math.max(maxBeachedDistance, stuckOnBallSim.getDistanceToBall());
    }

    // RECOVERED_TO_SCORE + BACKED_OFF: distance grew by >= 0.1 m across the first beached episode
    if (beachedAt >= 0.0 && recoveredToScoreAt < 0.0 && state == RobotState.SCORE) {
      recoveredToScoreAt = t;
      beachExitDistance = Math.max(maxBeachedDistance, stuckOnBallSim.getDistanceToBall());
      if (beachEnterDistance >= 0.0 && beachExitDistance - beachEnterDistance >= 0.1) {
        backedOffAt = t;
      }
    }
  }

  private String unmetAssertions() {
    var unmet = new ArrayList<String>();
    if (robotMovedAt < 0.0) unmet.add("ROBOT_MOVED");
    if (beachedAt < 0.0) unmet.add("BEACHED");
    if (hoodCompensatedAt < 0.0) unmet.add("HOOD_COMPENSATED");
    if (backedOffAt < 0.0) unmet.add("BACKED_OFF");
    if (recoveredToScoreAt < 0.0) unmet.add("RECOVERED_TO_SCORE");
    return String.join(", ", unmet);
  }
}
