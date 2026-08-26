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
  /** Angle from the horizontal to the deploy extension point. */
  private static final Rotation2d DEPLOY_ANGLE_FROM_HORIZONTAL = Rotation2d.fromDegrees(6.55);

  public static void log(
      Pose2d robotPose,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches) {
    var turretTranslation =
        new Translation3d(
            frc.robot.turret.TurretConfig.TURRET_TO_ROBOT.getX(), 0, Units.inchesToMeters(14.0));
    var hoodPivotTranslation =
        new Translation3d(
            frc.robot.turret.TurretConfig.TURRET_TO_ROBOT.getX(), 0, Units.inchesToMeters(18.85));

    var turretPose =
        new Pose3d(turretTranslation, new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));
    var shooterHoodPose =
        new Pose3d(hoodPivotTranslation, Rotation3d.kZero)
            .rotateAround(
                hoodPivotTranslation,
                new Rotation3d(
                    0,
                    -Math.toRadians(
                        shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                    0))
            .rotateAround(
                turretTranslation, new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));
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
