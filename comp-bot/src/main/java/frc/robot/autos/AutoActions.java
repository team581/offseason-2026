package frc.robot.autos;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.robot_manager.RobotManager;

/** Shared 2026 mechanism actions and transition conditions for autonomous state machines. */
public final class AutoActions {
  private static final double BUMP_CROSSING_SPEED_MPS = 4.0;
  private static final double BUMP_SHOT_WARMUP_DISTANCE_METERS = 2.171;
  private static final double SCORE_POSITION_TOLERANCE_METERS = 0.3;
  // The tracker samples a simulated tilt change on the next robot loop.
  private static final double SIM_FLAT_LEAD_METERS =
      BUMP_CROSSING_SPEED_MPS
          * (BumpCrossingTracker.FLAT_DEBOUNCE_SECONDS + TimedRobot.kDefaultPeriod);
  private static final double SIM_DOWNHILL_START_METERS = SIM_FLAT_LEAD_METERS + 0.4;
  private static final double SIM_CREST_START_METERS = SIM_DOWNHILL_START_METERS + 0.8;
  private static final double SIM_UPHILL_START_METERS = SIM_CREST_START_METERS + 0.8;

  /** The right and left normal autos use time in the shooting state for the first two shots. */
  public static boolean finishedShooting(TrailblazerAuto<?> auto) {
    return (auto.timeout(2.0) && !auto.robot.hopperManager.isShooting()) || auto.timeout(3.25);
  }

  /** Homes deploy and shooter hood at autonomous startup. */
  public static void homeSubsystems(RobotManager robot) {
    robot.homeDeployInAutoRequest();
    robot.homeShooterHoodRequest();
  }

  // VS Code's Diff view compiles the Git baseline auto against the current helpers.
  /** Prepares the robot for a Hub shot from the given blue-alliance position. */
  public static void prepareForBumpShot(RobotManager robot, Translation2d blueShootPosition) {
    robot.cancelIntakeRequest();
    robot.powerManager.idleRequest();
    robot.shooter.prepareScoreRequest(
        blueShootPosition.getDistance(FieldUtil.BLUE_HUB_TRANSLATION));
  }

  /** Homes mechanisms, enables auto intaking, and prioritizes flywheel/intake power on cycle 1. */
  public static void startFirstIntakeCycle(RobotManager robot) {
    homeSubsystems(robot);
    robot.intakeAutoRequest();
    robot.powerManager.firstAutoSegmentRequest();
  }

  /** Resets robot manager to idle with safe stow enabled before driving out for the next cycle. */
  public static void startNextCycle(RobotManager robot) {
    robot.idleRequest();
    robot.hopperManager.setWantsSafeStow(true);
  }

  /** Stops intaking and requests Hub scoring. */
  public static void startScoring(RobotManager robot) {
    robot.cancelIntakeRequest();
    robot.prepareScoreRequest();
  }

  static ChassisSpeeds bumpCrossingCommand(
      TickResult result, BumpCrossingState crossingState, Alliance alliance) {
    if (!hasBumpCrossingOverride(result, crossingState)) {
      return result.commandedFieldSpeeds();
    }
    // TB1 forced only field X during the crossing. Keep the follower's centering and heading.
    var planned = result.commandedFieldSpeeds();
    return new ChassisSpeeds(
        alliance == Alliance.Red ? BUMP_CROSSING_SPEED_MPS : -BUMP_CROSSING_SPEED_MPS,
        planned.vyMetersPerSecond,
        planned.omegaRadiansPerSecond);
  }

  static boolean finishedBumpCrossing(RobotManager robot) {
    // TB1's shot marker was active as soon as its one-point crossing segment started.
    return robot.localization.imu.bumpCrossingTracker.getState()
        == BumpCrossingState.FLAT_NOT_CROSSING;
  }

  static boolean hasBumpCrossingOverride(TickResult result, BumpCrossingState crossingState) {
    return crossingState != BumpCrossingState.FLAT_NOT_CROSSING
        && !result.diagnostic().equals("NONFINITE_STATE");
  }

  static void requestBumpCrossing(TrailblazerAuto<?> auto, Pose2d blueLandingPose) {
    var landing = auto.fieldPose(blueLandingPose);
    // The auto has already resolved the alliance for the entire run.
    auto.robot.localization.imu.bumpCrossingTracker.bumpCrossRequest(
        new Point(landing, landing), landing.getRotation().plus(Rotation2d.k180deg));
  }

  /** Score while holding the authored target within TB1's position tolerance. */
  static void scoreInPlace(TrailblazerAuto<?> auto, AutoSegment scoreSegment) {
    var goal = auto.fieldPose(scoreSegment.goals().get(0).pose()).getTranslation();
    var pose = auto.robot.localization.getPose().getTranslation();
    var status = auto.trailblazer.lastResult().status();
    if (pose.getDistance(goal) < SCORE_POSITION_TOLERANCE_METERS) {
      if (status != RuntimeStatus.IDLE) {
        auto.stopDriving();
      }
    } else if (status != RuntimeStatus.RUNNING) {
      auto.drive(scoreSegment);
    }
    auto.robot.prepareScoreRequest();
  }

  static AutoSegment scoreSegment(AutoSegment cycle) {
    var goals = cycle.goals();
    return new AutoSegment(
        cycle.id() + "-score", cycle.limits(), ImmutableList.of(goals.get(goals.size() - 1)));
  }

  static void whileCrossingBump(TrailblazerAuto<?> auto, Pose2d blueLandingPose) {
    auto.robot.shooter.prepareScoreRequest(BUMP_SHOT_WARMUP_DISTANCE_METERS);
    if (RobotBase.isSimulation()) {
      var landing = auto.fieldPose(blueLandingPose);
      var estimatedPose = auto.robot.localization.getPose();
      double remainingMeters =
          (landing.getX() - estimatedPose.getX()) * landing.getRotation().getCos();
      // Start the flat samples early enough to debounce while driving to the landing point.
      double tiltDegrees = 0.0;
      if (remainingMeters > SIM_FLAT_LEAD_METERS && remainingMeters <= SIM_DOWNHILL_START_METERS) {
        tiltDegrees = 7.5;
      } else if (remainingMeters > SIM_CREST_START_METERS
          && remainingMeters <= SIM_UPHILL_START_METERS) {
        tiltDegrees = -7.5;
      }
      auto.robot.localization.imu.setPitch(tiltDegrees);
      auto.robot.localization.imu.setRoll(tiltDegrees);
    }
  }

  private AutoActions() {}
}
