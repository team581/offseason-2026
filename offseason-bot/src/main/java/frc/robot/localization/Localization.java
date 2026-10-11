package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.TrustFactor;
import frc.robot.vision.Vision;
import frc.robot.vision.results.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Localization extends StateMachineSubsystem<LocalizationState> {
  public final Imu imu;
  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final TrustFactor trustFactor = new TrustFactor();

  private Pose2d robotPose = Pose2d.kZero;

  private double lastVisionLoopTimestamp = Double.NaN;

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.imu = imu;
    lastVisionLoopTimestamp = Timer.getFPGATimestamp();
    trustFactor.reset(lastVisionLoopTimestamp, false);
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

  public Optional<Pose2d> getVisionPoseAt(double fpgaTimestamp) {
    if (!Double.isFinite(fpgaTimestamp)) {
      return Optional.empty();
    }
    return drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(fpgaTimestamp));
  }

  public boolean isLost() {
    return trustFactor.isLost();
  }

  public boolean isTrustworthy() {
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

  private void ingestMeasurements(List<VisionMeasurement> results, double now) {
    // Recheck every history before the first estimator correction in this batch.
    var eligible = new ArrayList<VisionMeasurement>();
    for (var result : results) {
      if (vision.eligible(result, now) && getVisionPoseAt(result.timestamp()).isPresent())
        eligible.add(result);
    }
    for (var result : eligible) {
      drivetrain.addVisionMeasurement(
          result.pose(), Utils.fpgaToCurrentTime(result.timestamp()), result.standardDevs());
      vision.measurementFused(result);
      trustFactor.ingest(List.of(result));
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
    double now = vision.tickTime();
    var speeds = drivetrain.getState().Speeds;
    double elapsed =
        Double.isFinite(lastVisionLoopTimestamp) ? Math.max(0., now - lastVisionLoopTimestamp) : 0.;
    trustFactor.updateOdometry(
        MathHelpers.getLinearVelocity(speeds) * elapsed, elapsed, imu.collisionDetected());
    lastVisionLoopTimestamp = now;
    ingestMeasurements(vision.drainMeasurements(), now);
    robotPose = drivetrain.getState().Pose;
    DogLog.log("Vision/Spot/TrustEvidenceAgeS", trustFactor.evidenceAge(now));
  }
}
