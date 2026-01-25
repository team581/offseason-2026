package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.localization.TrustFactor;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.imu.Imu;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;

public class Localization extends StateMachineSubsystem<LocalizationState> {

  private static final double LATENCY_CONSTANT = 0.0;

  private final Swerve swerve;
  private final TunerSwerveDrivetrain drivetrain;
  private final Imu imu;
  private Pose2d robotPose = Pose2d.kZero;
  private final TrustFactor trustFactor = new TrustFactor();

  public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Imu imu) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.drivetrain = drivetrain;
    this.imu = imu;
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

  public void resetPose(Pose2d estimatedPose) {
    drivetrain.resetPose(estimatedPose);
  }

  @Override
  public void whileInState(LocalizationState currentState) {
    DogLog.log("Localization/EstimatedPose", getPose());
  }

  public void zeroGyro() {
    drivetrain.seedFieldCentric();
  }

  @Override
  protected void collectInputs() {
    robotPose = drivetrain.getState().Pose;

    trustFactor.update(robotPose, imu.collisionDetected());
  }
}
