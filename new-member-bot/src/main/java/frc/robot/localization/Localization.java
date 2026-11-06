package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.localization.TrustFactor;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.TagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.PracticeTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.ArrayList;
import java.util.List;

public class Localization extends StateMachineSubsystem<LocalizationState> {
  private static final DoubleSubscriber LATENCY_CONSTANT =
      DogLog.tunable("Localization/StaticLatencyAdjustment", 0.0);
  public final Imu imu;
  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final TrustFactor trustFactor = new TrustFactor();

  private Pose2d robotPose = Pose2d.kZero;

  private final List<TagResult> presentList = new ArrayList<>(3);

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.imu = imu;
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
    return trustFactor.isLost();
  }

  public boolean isTrustworthy() {
    return trustFactor.isTrustworthy();
  }

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
    trustFactor.seededPose();
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
    DogLog.log("Localization/TrustFactor", getTrustFactor());
    if (DriverStation.isAutonomous() && FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()) {
      DogLog.log(
          "Imu/BeachedRecovery/StuckOnBall",
          StuckOnBallRecovery.stuckOnBall(imu.getPitch(), imu.getRoll()));
    }
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
    trustFactor.reset();
  }

  private void ingestTagResult(List<TagResult> results) {
    DogLog.timestamp("Localization/IngestTagResult");
    var averageTimestamp = 0.0;
    for (TagResult result : results) {
      var visionPose = result.pose();
      averageTimestamp += result.timestamp();
      swerve.drivetrain.addVisionMeasurement(
          visionPose,
          Utils.fpgaToCurrentTime(
              Math.min(
                  Timer.getFPGATimestamp() - (LATENCY_CONSTANT.get() / 1000),
                  result.timestamp() - (LATENCY_CONSTANT.get() / 1000))),
          result.standardDevs());
    }
    averageTimestamp = averageTimestamp / results.size();
    trustFactor.ingestTagResult(getPose(averageTimestamp), results);
  }

  @Override
  protected void collectInputs() {
    presentList.clear();
    vision.getShooterLimelightTagResult().ifPresent(presentList::add);
    vision.getLeftLimelightTagResult().ifPresent(presentList::add);
    vision.getRightLimelightTagResult().ifPresent(presentList::add);

    ingestTagResult(presentList);

    robotPose = swerve.getDriveState().Pose;
    vision.setEstimatedPoseAngle(robotPose.getRotation().getDegrees());
    trustFactor.update(robotPose, Swerve.TRANSLATION_STD_DEV, imu.collisionDetected());
  }
}
