package frc.robot.deploy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** CAD-derived parallel-linkage motion, driven directly by the measured arm angle. */
public final class DeployKinematics {
  public static final double FULL_DEPLOY_ANGLE_DEGREES = 88.4;

  // Robot coordinates in meters, recovered from the in/out STEP assembly placements.
  public static final Translation3d UPPER_ARM_PIVOT =
      new Translation3d(0.146050000, 0.0, 0.262638525);
  public static final Translation3d LOWER_ARM_PIVOT =
      new Translation3d(0.230872745, 0.0, 0.203200000);

  // Vector from either chassis pivot to its carriage joint when fully retracted.
  private static final Translation3d RETRACTED_LINK_VECTOR =
      new Translation3d(-0.066188841, 0.0, 0.277506655);

  /** Poses ordered as carriage displacement, upper-arm pivot, lower-arm pivot. */
  public static Pose3d[] componentPoses(double angleDegrees) {
    var sweep =
        MathUtil.clamp(FULL_DEPLOY_ANGLE_DEGREES - angleDegrees, 0.0, FULL_DEPLOY_ANGLE_DEGREES);
    var rotation = new Rotation3d(0.0, Math.toRadians(sweep), 0.0);
    // The parallel links keep the carriage level while their attachment points follow an arc.
    var displacement = RETRACTED_LINK_VECTOR.rotateBy(rotation).minus(RETRACTED_LINK_VECTOR);
    return new Pose3d[] {
      new Pose3d(displacement, Rotation3d.kZero),
      new Pose3d(UPPER_ARM_PIVOT, rotation),
      new Pose3d(LOWER_ARM_PIVOT, rotation)
    };
  }

  private DeployKinematics() {}
}
