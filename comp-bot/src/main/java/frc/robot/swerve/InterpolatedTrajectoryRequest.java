package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jspecify.annotations.Nullable;

/** Samples a Trailblazer velocity step on the Phoenix odometry thread. */
final class InterpolatedTrajectoryRequest implements SwerveRequest {
  private record Ramp(
      double timestamp,
      double duration,
      double startVx,
      double startVy,
      double startOmega,
      double endVx,
      double endVy,
      double endOmega) {}

  // The robot thread publishes complete snapshots. Only the odometry thread mutates the request.
  private volatile @Nullable Ramp ramp;
  private final ChassisSpeeds sampledSpeeds = new ChassisSpeeds();
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0)
          .withRotationalDeadband(0);

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
    sample(parameters.timestamp, sampledSpeeds);
    return fieldCentric
        .withVelocityX(sampledSpeeds.vxMetersPerSecond)
        .withVelocityY(sampledSpeeds.vyMetersPerSecond)
        .withRotationalRate(sampledSpeeds.omegaRadiansPerSecond)
        .apply(parameters, modules);
  }

  void clear() {
    ramp = null;
  }

  boolean hasStep() {
    return ramp != null;
  }

  void sample(double timestamp, ChassisSpeeds output) {
    var active = ramp;
    if (active == null) {
      output.vxMetersPerSecond = 0;
      output.vyMetersPerSecond = 0;
      output.omegaRadiansPerSecond = 0;
      return;
    }
    double fraction = MathUtil.clamp((timestamp - active.timestamp()) / active.duration(), 0, 1);
    output.vxMetersPerSecond = MathUtil.interpolate(active.startVx(), active.endVx(), fraction);
    output.vyMetersPerSecond = MathUtil.interpolate(active.startVy(), active.endVy(), fraction);
    output.omegaRadiansPerSecond =
        MathUtil.interpolate(active.startOmega(), active.endOmega(), fraction);
  }

  void setStep(VelocityStep step, ChassisSpeeds target) {
    var start = step.startingFieldSpeeds();
    ramp =
        new Ramp(
            step.timestampSeconds(),
            step.durationSeconds(),
            start.vxMetersPerSecond,
            start.vyMetersPerSecond,
            start.omegaRadiansPerSecond,
            target.vxMetersPerSecond,
            target.vyMetersPerSecond,
            target.omegaRadiansPerSecond);
  }
}
