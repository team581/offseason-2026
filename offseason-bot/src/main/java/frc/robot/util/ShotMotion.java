package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Short-horizon drive response prediction, always anchored to measured field-relative motion. */
public record ShotMotion(Pose2d pose, ChassisSpeeds speeds) {
  private static final double STEP_SECONDS = 0.005;

  public static ShotMotion predict(
      Pose2d pose, ChassisSpeeds measured, ChassisSpeeds requested, double seconds) {
    var velocity = new Translation2d(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
    var target = new Translation2d(requested.vxMetersPerSecond, requested.vyMetersPerSecond);
    double omega = measured.omegaRadiansPerSecond;
    var predictedPose = pose;
    // Integrate a bounded first-order response instead of assuming the chassis follows the stick.
    double remaining = MathUtil.clamp(seconds, 0.0, 0.25);
    while (remaining > 1e-9) {
      double dt = Math.min(STEP_SECONDS, remaining);
      var delta = target.minus(velocity).times(1.0 - Math.exp(-dt / 0.15));
      if (delta.getNorm() > 5.0 * dt) {
        delta = delta.times(5.0 * dt / delta.getNorm());
      }
      double angularDelta =
          MathUtil.clamp(
              (requested.omegaRadiansPerSecond - omega) * (1.0 - Math.exp(-dt / 0.10)),
              -20.0 * dt,
              20.0 * dt);
      predictedPose =
          new Pose2d(
              predictedPose.getTranslation().plus(velocity.plus(delta.times(0.5)).times(dt)),
              predictedPose
                  .getRotation()
                  .plus(Rotation2d.fromRadians((omega + angularDelta * 0.5) * dt)));
      velocity = velocity.plus(delta);
      omega += angularDelta;
      remaining -= dt;
    }
    return new ShotMotion(
        predictedPose, new ChassisSpeeds(velocity.getX(), velocity.getY(), omega));
  }
}
