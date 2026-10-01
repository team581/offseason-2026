package frc.robot.robot_manager;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.deploy.DeployKinematics;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.turret.TurretConfig;

public final class MechanismVisualizer {
  private static final Translation3d SHOOTER_HOOD_PIVOT_POINT =
      new Translation3d(0.105664019, 0.0, 0.083120893);

  private static final double TURRET_HEIGHT_METERS = Units.inchesToMeters(14.750);

  /** Keeps inactive colored components away from the robot instead of at its origin. */
  private static final Pose3d HIDDEN_COMPONENT_POSE =
      new Pose3d(0.0, 0.0, -100.0, Rotation3d.kZero);

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
   * Builds poses in the fixed AdvantageScope component order: green hood, green turret, intake
   * carriage, yellow hood, blue turret, upper deploy arms, lower deploy arms.
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
    var deployPoses = DeployKinematics.componentPoses(deployLengthInches);
    var active = robotState == RobotState.SCORE || robotState == RobotState.FEED;
    return active
        ? new Pose3d[] {
          shooterHoodPose,
          turretPose,
          deployPoses[0],
          HIDDEN_COMPONENT_POSE,
          HIDDEN_COMPONENT_POSE,
          deployPoses[1],
          deployPoses[2]
        }
        : new Pose3d[] {
          HIDDEN_COMPONENT_POSE,
          HIDDEN_COMPONENT_POSE,
          deployPoses[0],
          shooterHoodPose,
          turretPose,
          deployPoses[1],
          deployPoses[2]
        };
  }

  private MechanismVisualizer() {}
}
