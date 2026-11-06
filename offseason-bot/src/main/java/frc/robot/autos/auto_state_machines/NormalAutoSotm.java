package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretConfig;

final class NormalAutoSotm {
  static final double HARD_STOP_CLEARANCE_DEGREES = 5.0;

  static boolean hasClearance(Pose2d redPose) {
    double angle =
        TurretCalculator.calculateTurretAimingAngle(
            redPose, FieldUtil.HUB_POSE.redPose().getTranslation());
    double unwrapped = MathUtil.inputModulus(angle, -360.0, 0.0);
    return unwrapped >= TurretConfig.MIN_ANGLE + HARD_STOP_CLEARANCE_DEGREES
        && unwrapped <= TurretConfig.MAX_ANGLE - HARD_STOP_CLEARANCE_DEGREES;
  }

  static Point shootingGoal(Translation2d start, Translation2d end, boolean clockwise) {
    double direction = clockwise ? -1.0 : 1.0;
    double reachable = 0.0;
    double unreachable = 180.0;
    // Find the largest fixed turn that is safe even if rotation finishes before translation.
    // For a straight translation at a fixed heading, the extreme hub bearings are at its ends.
    for (int iteration = 0; iteration < 40; iteration++) {
      double candidate = (reachable + unreachable) / 2.0;
      var heading = Rotation2d.fromDegrees(direction * candidate);
      if (hasClearance(new Pose2d(start, heading)) && hasClearance(new Pose2d(end, heading))) {
        reachable = candidate;
      } else {
        unreachable = candidate;
      }
    }
    return Point.ofRed(new Pose2d(end, Rotation2d.fromDegrees(direction * reachable)));
  }

  private NormalAutoSotm() {}
}
