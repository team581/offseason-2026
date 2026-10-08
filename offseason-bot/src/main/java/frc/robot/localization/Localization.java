package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.localization.TrustFactor;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.CaptureHistory;
import com.team581.vision.LimelightVisionIO;
import com.team581.vision.SpotVisionIO;
import com.team581.vision.VisionDiagnostics;
import com.team581.vision.VisionMeasurement;
import com.team581.vision.VisionPipeline;
import com.team581.vision.VisionProcessor;
import com.team581.vision.VisionTrust;
import com.team581.vision.results.OptionalTagResult;
import com.team581.vision.results.TagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.SpotCameraProfiles;
import frc.robot.vision.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class Localization extends StateMachineSubsystem<LocalizationState> {
  private static final DoubleSubscriber LATENCY_CONSTANT =
      DogLog.tunable("Localization/StaticLatencyAdjustment", 0.0);
  public final Imu imu;
  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final TrustFactor trustFactor = new TrustFactor();

  private Pose2d robotPose = Pose2d.kZero;

  private final CaptureHistory captureHistory = new CaptureHistory(this::getVisionPoseAt);
  private final VisionPipeline visionPipeline;
  private final VisionTrust revisedTrust = new VisionTrust();
  private double lastVisionLoopTimestamp = Double.NaN;

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.vision = vision;
    this.drivetrain = drivetrain;
    this.imu = imu;
    var profile = SpotCameraProfiles.profile();
    var io = new SpotVisionIO(NetworkTableInstance.getDefault(), profile);
    visionPipeline =
        new VisionPipeline(
            io,
            profile,
            new VisionProcessor(
                profile,
                SpotCameraProfiles.mounts(),
                captureHistory,
                VisionProcessor.Policy.conservative()),
            List.of(
                new LimelightVisionIO(
                    "Limelight/shooter",
                    vision::getShooterLimelightTagResult,
                    () -> LATENCY_CONSTANT.get() / 1000.),
                new LimelightVisionIO(
                    "Limelight/left",
                    vision::getLeftLimelightTagResult,
                    () -> LATENCY_CONSTANT.get() / 1000.),
                new LimelightVisionIO(
                    "Limelight/right",
                    vision::getRightLimelightTagResult,
                    () -> LATENCY_CONSTANT.get() / 1000.)),
            new VisionDiagnostics());
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
    return revisedTrustEnabled() ? revisedTrust.get() : trustFactor.get();
  }

  public Optional<Pose2d> getVisionPoseAt(double fpgaTimestamp) {
    if (!Double.isFinite(fpgaTimestamp)) {
      return Optional.empty();
    }
    return drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(fpgaTimestamp));
  }

  public boolean isLost() {
    return revisedTrustEnabled() ? revisedTrust.isLost() : trustFactor.isLost();
  }

  public boolean isTrustworthy() {
    return revisedTrustEnabled() ? revisedTrust.isTrustworthy() : trustFactor.isTrustworthy();
  }

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
    trustFactor.seededPose();
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
    trustFactor.reset();
    resetVisionHistory(false);
  }

  private void ingestMeasurements(List<VisionMeasurement> results, double now) {
    if (results.isEmpty()) {
      return;
    }
    List<VisionMeasurement> eligible = new ArrayList<>();
    for (var result : results) {
      var historical = getVisionPoseAt(result.timestamp());
      if (now - result.timestamp() > .25
          || result.timestamp() > now + .005
          || historical.isEmpty()) {
        continue;
      }
      if (result.source().startsWith("Limelight/")) {
        double innovation =
            historical.orElseThrow().getTranslation().getDistance(result.pose().getTranslation());
        eligible.add(
            new VisionMeasurement(
                result.source(),
                result.frameId(),
                result.pose(),
                result.timestamp(),
                result.stdX(),
                result.stdY(),
                result.stdHeading(),
                result.tagIds(),
                innovation,
                innovation / Math.hypot(result.stdX(), result.stdY())));
      } else {
        eligible.add(result);
      }
    }
    if (eligible.isEmpty()) {
      return;
    }
    // Capture-time residuals were computed before any estimator update. Legacy
    // trust remains the default; revised trust requires both reviewed flags.
    if (revisedTrustEnabled()) {
      revisedTrust.ingest(eligible);
    } else {
      List<TagResult> legacy = new ArrayList<>();
      for (var result : eligible) {
        legacy.add(
            new OptionalTagResult()
                .update(result.pose(), result.timestamp(), result.standardDevs())
                .orElseThrow());
      }
      double average =
          eligible.stream().mapToDouble(VisionMeasurement::timestamp).average().orElseThrow();
      trustFactor.ingestTagResult(getPose(average), legacy);
    }
    for (var result : eligible) {
      drivetrain.addVisionMeasurement(
          result.pose(), Utils.fpgaToCurrentTime(result.timestamp()), result.standardDevs());
      visionPipeline.measurementFused(result);
    }
  }

  private void resetVisionHistory(boolean seeded) {
    double now = Timer.getFPGATimestamp();
    visionPipeline.reset(now);
    captureHistory.reset();
    revisedTrust.reset(now, seeded);
    lastVisionLoopTimestamp = now;
  }

  private boolean revisedTrustEnabled() {
    return SpotCameraProfiles.profile().getRevisedTrustEnabled()
        && FeatureFlags.SPOT_REVISED_TRUST.getAsBoolean();
  }

  @Override
  protected void collectInputs() {
    double now = Timer.getFPGATimestamp();
    var speeds = swerve.getRobotRelativeSpeeds();
    captureHistory.add(
        now,
        new VisionProcessor.CaptureState(
            swerve.getDriveState().Pose,
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
            speeds.omegaRadiansPerSecond,
            Math.toRadians(imu.getRoll()),
            Math.toRadians(imu.getPitch()),
            0.));
    var measurements = visionPipeline.collect(now, FeatureFlags.SPOT_FUSION.getAsBoolean(), false);
    ingestMeasurements(measurements, now);
    visionPipeline.publishDecisions(now);
    double elapsed =
        Double.isFinite(lastVisionLoopTimestamp) ? Math.max(0., now - lastVisionLoopTimestamp) : 0.;
    revisedTrust.updateOdometry(
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * elapsed,
        elapsed,
        imu.collisionDetected());
    lastVisionLoopTimestamp = now;
    DogLog.log("Vision/Spot/ShadowAccepted", visionPipeline.shadowMeasurements().size());
    DogLog.log("Vision/Spot/TrustEvidenceAgeS", revisedTrust.evidenceAge(now));

    robotPose = swerve.getDriveState().Pose;
    vision.setEstimatedPoseAngle(robotPose.getRotation().getDegrees());
    trustFactor.update(robotPose, Swerve.TRANSLATION_STD_DEV, imu.collisionDetected());
  }
}
