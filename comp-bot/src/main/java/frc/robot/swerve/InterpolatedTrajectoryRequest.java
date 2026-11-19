package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.CompTrailblazer;

/** The publisher owns the ramp timeline. Phoenix samples a single immutable ramp per callback. */
final class InterpolatedTrajectoryRequest implements SwerveRequest {
  private record Ramp(
      VelocityStep step,
      double x,
      double y,
      double w,
      double endX,
      double endY,
      double endW,
      double linearDuration,
      double angularDuration,
      double linearRate,
      double angularRate) {
    private double fraction(double time, double duration) {
      return duration == 0 || time >= step.startTime() + duration
          ? 1
          : MathUtil.clamp((time - step.startTime()) / duration, 0, 1);
    }

    void sample(double time, ChassisSpeeds v, ChassisSpeeds a) {
      double translation = fraction(time, linearDuration);
      double rotation = fraction(time, angularDuration);
      v.vxMetersPerSecond = MathUtil.interpolate(x, endX, translation);
      v.vyMetersPerSecond = MathUtil.interpolate(y, endY, translation);
      v.omegaRadiansPerSecond = MathUtil.interpolate(w, endW, rotation);
      boolean moving = translation < 1 && time >= step.startTime();
      a.vxMetersPerSecond = moving ? (endX - x) / linearDuration : 0;
      a.vyMetersPerSecond = moving ? (endY - y) / linearDuration : 0;
      a.omegaRadiansPerSecond =
          rotation < 1 && time >= step.startTime() ? (endW - w) / angularDuration : 0;
    }
  }

  private static final double EPSILON = 1e-9;

  private static double slewDuration(
      VelocityStep step,
      boolean continuous,
      boolean holding,
      double previousEnd,
      double distance,
      double rate) {
    double remaining = Math.max(0, previousEnd - step.startTime());
    if (holding) return remaining;
    double duration =
        rate < EPSILON ? step.durationSeconds() : Math.max(step.durationSeconds(), distance / rate);
    // For adjacent steps, finishing the old slew and executing the new one is feasible.
    // Their average over that interval cannot exceed the larger of the two slew rates.
    // This keeps a tiny new correction from stretching old unfinished motion indefinitely.
    return continuous ? Math.min(duration, remaining + step.durationSeconds()) : duration;
  }

  private final WheelForceCalculator calculator =
      new WheelForceCalculator(
          CompTrailblazer.DRIVE_SPEC.modulePositionsMeters().toArray(Translation2d[]::new),
          CompTrailblazer.DRIVE_SPEC.massKg(),
          CompTrailblazer.DRIVE_SPEC.yawMoiKgMetersSq());
  private final SwerveRequest.ApplyRobotSpeeds request =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  final DriveRequestSamples samples = new DriveRequestSamples("timestamped_velocity_and_force");
  private final ChassisSpeeds velocity = new ChassisSpeeds();
  private final ChassisSpeeds acceleration = new ChassisSpeeds();

  private volatile Ramp ramp;

  InterpolatedTrajectoryRequest() {
    setDirect(new ChassisSpeeds());
  }

  @Override
  public StatusCode apply(SwerveControlParameters p, SwerveModule<?, ?, ?>... modules) {
    var active = ramp;
    active.sample(p.timestamp, velocity, acceleration);
    var heading = p.currentPose.getRotation();
    var ff = forces(acceleration, heading);
    var status =
        request
            .withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(velocity, heading))
            .withWheelForceFeedforwardsX(ff.x_newtons)
            .withWheelForceFeedforwardsY(ff.y_newtons)
            .apply(p, modules);
    samples.capture(
        p.timestamp,
        active.step.sequence(),
        velocity,
        acceleration,
        active.linearDuration - active.step.durationSeconds(),
        active.angularDuration - active.step.durationSeconds(),
        heading.getRadians(),
        status.value,
        ff);
    return status;
  }

  Feedforwards forces(ChassisSpeeds fieldAcceleration, Rotation2d heading) {
    var a = ChassisSpeeds.fromFieldRelativeSpeeds(fieldAcceleration, heading);
    return calculator.calculate(a.vxMetersPerSecond, a.vyMetersPerSecond, a.omegaRadiansPerSecond);
  }

  boolean hasStep() {
    return ramp.step.durationSeconds() > 0;
  }

  void sample(double time, ChassisSpeeds v, ChassisSpeeds a) {
    ramp.sample(time, v, a);
  }

  void setDirect(ChassisSpeeds v) {
    ramp =
        new Ramp(
            new VelocityStep(v, 0),
            v.vxMetersPerSecond,
            v.vyMetersPerSecond,
            v.omegaRadiansPerSecond,
            v.vxMetersPerSecond,
            v.vyMetersPerSecond,
            v.omegaRadiansPerSecond,
            0,
            0,
            0,
            0);
  }

  void setStep(VelocityStep step, ChassisSpeeds end) {
    var old = ramp;
    if (old.step.durationSeconds() > 0 && step.sequence() <= old.step.sequence()) return;
    var start = new ChassisSpeeds(step.startVx(), step.startVy(), step.startOmega());
    double duration = step.durationSeconds();
    double linear =
        Math.hypot(end.vxMetersPerSecond - step.startVx(), end.vyMetersPerSecond - step.startVy())
            / duration;
    double angular = Math.abs(end.omegaRadiansPerSecond - step.startOmega()) / duration;
    double linearDuration = duration;
    double angularDuration = duration;
    if (old.step.durationSeconds() > 0) {
      old.sample(step.startTime(), start, new ChassisSpeeds());
      boolean sameTranslation =
          end.vxMetersPerSecond == old.endX
              && end.vyMetersPerSecond == old.endY
              && step.startVx() == old.endX
              && step.startVy() == old.endY;
      boolean sameRotation = end.omegaRadiansPerSecond == old.endW && step.startOmega() == old.endW;
      if (linear < EPSILON) linear = old.linearRate;
      if (angular < EPSILON) angular = old.angularRate;
      linearDuration =
          slewDuration(
              step,
              step.startVx() == old.endX && step.startVy() == old.endY,
              sameTranslation,
              old.step.startTime() + old.linearDuration,
              Math.hypot(
                  end.vxMetersPerSecond - start.vxMetersPerSecond,
                  end.vyMetersPerSecond - start.vyMetersPerSecond),
              linear);
      angularDuration =
          slewDuration(
              step,
              step.startOmega() == old.endW,
              sameRotation,
              old.step.startTime() + old.angularDuration,
              Math.abs(end.omegaRadiansPerSecond - start.omegaRadiansPerSecond),
              angular);
    }
    ramp =
        new Ramp(
            step,
            start.vxMetersPerSecond,
            start.vyMetersPerSecond,
            start.omegaRadiansPerSecond,
            end.vxMetersPerSecond,
            end.vyMetersPerSecond,
            end.omegaRadiansPerSecond,
            linearDuration,
            angularDuration,
            linear,
            angular);
  }
}
