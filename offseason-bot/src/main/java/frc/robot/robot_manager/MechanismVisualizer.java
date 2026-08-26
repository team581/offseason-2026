package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.turret.TurretConfig;

public final class MechanismVisualizer {
  private static final Translation3d SHOOTER_HOOD_PIVOT_POINT =
      new Translation3d(Units.inchesToMeters(3.0), 0.0, Units.inchesToMeters(6.0));

  private static final double TURRET_HEIGHT_METERS = Units.inchesToMeters(14.0);

  /** Keeps inactive AdvantageScope components away from the robot instead of at its origin. */
  private static final Pose3d HIDDEN_COMPONENT_POSE =
      new Pose3d(0.0, 0.0, -100.0, Rotation3d.kZero);

  /** Angle from the horizontal to the deploy extension point. */
  private static final Rotation2d DEPLOY_ANGLE_FROM_HORIZONTAL = Rotation2d.fromDegrees(15.327113);

  public static void log(
      RobotState robotState,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches) {
    DogLog.log(
        "SuperstructureVisualization/Components",
        buildComponentPoses(
            robotState, turretAngleDegrees, shooterHoodAngleDegrees, deployLengthInches));
  }

  /**
   * Builds poses in the fixed AdvantageScope component order: green hood, green turret, deploy,
   * yellow hood, blue turret.
   */
  static Pose3d[] buildComponentPoses(
      RobotState robotState,
      double turretAngleDegrees,
      double shooterHoodAngleDegrees,
      double deployLengthInches) {
    var turretTranslation =
        new Translation3d(
            TurretConfig.TURRET_TO_ROBOT.getX(),
            TurretConfig.TURRET_TO_ROBOT.getY(),
            TURRET_HEIGHT_METERS);

    var turretPose =
        new Pose3d(turretTranslation, new Rotation3d(0, 0, Math.toRadians(turretAngleDegrees)));
    var shooterHoodPose =
        turretPose.transformBy(
            new Transform3d(
                SHOOTER_HOOD_PIVOT_POINT,
                new Rotation3d(
                    0,
                    Math.toRadians(
                        shooterHoodAngleDegrees - ShooterHoodConfig.ANGLE_FROM_HORIZONTAL),
                    0)));
    var deployPose =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(deployLengthInches), 0, 0)
                .rotateBy(new Rotation3d(0, DEPLOY_ANGLE_FROM_HORIZONTAL.getRadians(), 0)),
            Rotation3d.kZero);

    var isActiveScoringOrFeeding = robotState == RobotState.SCORE || robotState == RobotState.FEED;
    return isActiveScoringOrFeeding
        ? new Pose3d[] {
          shooterHoodPose, turretPose, deployPose, HIDDEN_COMPONENT_POSE, HIDDEN_COMPONENT_POSE
        }
        : new Pose3d[] {
          HIDDEN_COMPONENT_POSE, HIDDEN_COMPONENT_POSE, deployPose, shooterHoodPose, turretPose
        };
  }

  private MechanismVisualizer() {}
}
