package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter_hood.ShooterHoodConfig;

public final class MechanismVisualizer {
  /**
   * If (0, 0, 0) is the robot origin, this translation defines the point that the shooter hood
   * pivots around.
   */
  private static final Translation3d SHOOTER_HOOD_PIVOT_POINT =
      new Translation3d(
          -Units.inchesToMeters(11.35), Units.inchesToMeters(13.85), Units.inchesToMeters(18.85));

  /** Angle from the horizontal to the deploy extension point. */
  private static final Rotation2d DEPLOY_ANGLE_FROM_HORIZONTAL = Rotation2d.fromDegrees(6.55);

  public static void log(
      Pose2d robotPose,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches) {
    var turretPose =
        new Pose3d(Translation3d.kZero, new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));
    var shooterHoodPose =
        Pose3d.kZero
            .rotateAround(
                SHOOTER_HOOD_PIVOT_POINT,
                new Rotation3d(
                    0,
                    -Math.toRadians(
                        shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                    0))
            .rotateBy(turretPose.getRotation());
    var deployPose =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(deployLengthInches), 0, 0)
                .rotateBy(new Rotation3d(0, DEPLOY_ANGLE_FROM_HORIZONTAL.getRadians(), 0)),
            Rotation3d.kZero);

    DogLog.log(
        "SuperstructureVisualization/Components",
        new Pose3d[] {turretPose, shooterHoodPose, deployPose, Pose3d.kZero, Pose3d.kZero});
  }

  private MechanismVisualizer() {}
}
