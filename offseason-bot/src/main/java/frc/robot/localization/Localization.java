package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionMeasurement;
import java.util.List;

public class Localization extends StateMachineSubsystem<LocalizationState> {
  // Heading comes from the gyro; vision only corrects translation.
  private static final double VISION_HEADING_STD = 1e6;

  public final Imu imu;
  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Vision vision;

  private final TrustFactor trustFactor = new TrustFactor();

  private Pose2d robotPose = Pose2d.kZero;

  private double lastVisionLoopTimestamp = Double.NaN;

  // Set from the tunable's notifier thread, applied on the main loop
  private volatile double pendingHeadingResetDegrees = Double.NaN;

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.imu = imu;
    lastVisionLoopTimestamp = Timer.getFPGATimestamp();
    trustFactor.reset(lastVisionLoopTimestamp, false);

    DogLog.tunable(
        "Localization/ResetHeadingDegrees", 0.0, degrees -> pendingHeadingResetDegrees = degrees);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    var current = getPose();
    var velocity = swerve.getFieldRelativeSpeeds();
    var x = current.getX() + velocity.vxMetersPerSecond * lookahead;
    var y = current.getY() + velocity.vyMetersPerSecond * lookahead;
    var theta =
        current
            .getRotation()
            .plus(Rotation2d.fromRadians(velocity.omegaRadiansPerSecond * lookahead));

    return new Pose2d(x, y, theta);
  }

  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return drivetrain.samplePoseAt(newTimestamp).orElseGet(this::getPose);
  }

  public double getTrustFactor() {
    return trustFactor.get();
  }

  public boolean isLost() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return false;
    }
    return trustFactor.isLost();
  }

  public boolean isTrustworthy() {
    // Bypass trust factor checks in simulation, since we don't have simulated cameras
    if (RobotBase.isSimulation()) {
      return true;
    }
    return trustFactor.isTrustworthy();
  }

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
    resetVisionHistory(true);
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
    DogLog.log("Localization/TrustFactor", getTrustFactor());
    DogLog.log("Vision/Spot/PoseTrustworthy", isTrustworthy());

    if (DriverStation.isAutonomous()
        && (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
            || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean())) {
      DogLog.log(
          "Imu/RobotPoseWithTilt",
          new Pose3d(
              new Translation3d(robotPose.getX(), robotPose.getY(), 0.0),
              new Rotation3d(
                  Math.toRadians(imu.getRoll()),
                  Math.toRadians(imu.getPitch()),
                  robotPose.getRotation().getRadians())));
      DogLog.log(
          "Imu/BeachedRecovery/RecoveryPose",
          StuckOnBallRecovery.getRecoveryPose(
              robotPose,
              Rotation2d.fromDegrees(imu.getPitch()),
              Rotation2d.fromDegrees(imu.getRoll())));
      DogLog.log(
          "Imu/BeachedRecovery/StuckOnBall",
          StuckOnBallRecovery.stuckOnBall(imu.getPitch(), imu.getRoll()));
    }
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
    resetVisionHistory(false);
  }

  private void ingestMeasurements(List<VisionMeasurement> results) {
    for (var result : results) {
      double std = result.std() * vision.correlationFactor();
      drivetrain.addVisionMeasurement(
          result.pose(),
          Utils.fpgaToCurrentTime(result.timestamp()),
          VecBuilder.fill(std, std, VISION_HEADING_STD));
      trustFactor.ingest(result);
    }
  }

  private void resetVisionHistory(boolean seeded) {
    double now = Timer.getFPGATimestamp();
    vision.reset(now);
    trustFactor.reset(now, seeded);
    lastVisionLoopTimestamp = now;
    robotPose = drivetrain.getState().Pose;
  }

  @Override
  protected void collectInputs() {
    double headingResetDegrees = pendingHeadingResetDegrees;
    if (Double.isFinite(headingResetDegrees)) {
      pendingHeadingResetDegrees = Double.NaN;
      drivetrain.resetRotation(Rotation2d.fromDegrees(headingResetDegrees));
      resetVisionHistory(false);
    }

    double now = vision.tickTime();
    var speeds = drivetrain.getState().Speeds;
    double elapsed =
        Double.isFinite(lastVisionLoopTimestamp) ? Math.max(0., now - lastVisionLoopTimestamp) : 0.;
    trustFactor.updateOdometry(MathHelpers.getLinearVelocity(speeds) * elapsed, elapsed);
    lastVisionLoopTimestamp = now;
    ingestMeasurements(vision.drainMeasurements());
    robotPose = drivetrain.getState().Pose;
    DogLog.log("Vision/Spot/TrustEvidenceAgeS", trustFactor.evidenceAge(now));
  }
}
