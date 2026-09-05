package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretConfig;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class AimParameterUtilTest {
  private static final double DELTA = 1e-8;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void dynamicSolutionAndLookaheadUseProjectedRobotPose() {
    var robotPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(25.0));
    var speeds = new ChassisSpeeds(0.7, -0.4, 0.6);
    var dynamic = AimParameterUtil.getScoringParameters(robotPose, speeds);
    var stationary = AimParameterUtil.getStaticScoringParameters(robotPose, speeds);

    assertNotEquals(stationary.turretAngle(), dynamic.turretAngle(), 1e-5);

    var turretPose = TurretCalculator.getTurretPose(robotPose);
    var turretSpeeds =
        TurretCalculator.getTurretChassisSpeeds(speeds, robotPose.getRotation().getDegrees());
    var compensatedGoal =
        new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF)
            .getSeparatedVelocityCompensatedGoalWithEffectiveTof(
                turretPose.getTranslation(), FieldUtil.HUB_POSE.getTranslation(), turretSpeeds)
            .fullyCompensatedGoal();
    var futureRobotPose =
        new Pose2d(
            robotPose
                .getTranslation()
                .plus(
                    new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
                        .times(0.5)),
            robotPose
                .getRotation()
                .plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.5)));

    assertEquals(
        TurretCalculator.calculateTurretAimingAngle(futureRobotPose, compensatedGoal),
        dynamic.upcomingTurretAngle(),
        DELTA);
  }

  @Test
  void fallbackAnglesAreRobotRelativeAndStableForLookahead() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    var blue = AimParameterUtil.getFallbackFeedingParameters(Rotation2d.fromDegrees(30.0));
    assertEquals(150.0, blue.turretAngle(), DELTA);
    assertEquals(blue.turretAngle(), blue.upcomingTurretAngle(), DELTA);

    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    var red = AimParameterUtil.getFallbackFeedingParameters(Rotation2d.fromDegrees(30.0));
    assertEquals(-30.0, red.turretAngle(), DELTA);
    assertEquals(red.turretAngle(), red.upcomingTurretAngle(), DELTA);
  }

  @Test
  void lateralAndRotationalMotionProduceNegativeTurretFeedforward() {
    var offset = TurretConfig.TURRET_TO_ROBOT.getTranslation();
    var goal = FieldUtil.HUB_POSE.getTranslation();
    var robotPose =
        new Pose2d(goal.minus(offset).minus(new Translation2d(4.0, 0.0)), Rotation2d.kZero);

    var lateral =
        AimParameterUtil.getScoringParameters(robotPose, new ChassisSpeeds(0.0, 1.0, 0.0));
    var rotating =
        AimParameterUtil.getScoringParameters(robotPose, new ChassisSpeeds(0.0, 0.0, 1.0));

    assertThat(lateral.turretFeedForwardRadians()).isLessThan(0.0);
    assertThat(rotating.turretFeedForwardRadians()).isLessThan(-1.0);
  }

  @Test
  void staticFeedingUsesTwentyInchTolerance() {
    var robotPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(-20.0));
    var parameters =
        AimParameterUtil.getStaticFeedingParameters(
            FeedLocation.RIGHT, robotPose, new ChassisSpeeds());

    assertEquals(
        TurretCalculator.getGoalCentricTurretTolerance(
            FeedLocation.RIGHT.getTranslation(), robotPose, Units.inchesToMeters(20.0)),
        parameters.turretTolerance(),
        DELTA);
  }

  @Test
  void staticScoringUsesCornerTurretPoseAndFiveInchTolerance() {
    var robotPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(35.0));
    var parameters = AimParameterUtil.getStaticScoringParameters(robotPose, new ChassisSpeeds());
    var turretPose = TurretCalculator.getTurretPose(robotPose);
    var goal = FieldUtil.HUB_POSE.getTranslation();

    assertEquals(
        TurretCalculator.calculateTurretAimingAngle(robotPose, goal),
        parameters.turretAngle(),
        DELTA);
    assertEquals(turretPose.getTranslation().getDistance(goal), parameters.distance(), DELTA);
    assertEquals(
        TurretCalculator.getGoalCentricTurretTolerance(goal, robotPose, Units.inchesToMeters(5.0)),
        parameters.turretTolerance(),
        DELTA);
    assertEquals(parameters.turretAngle(), parameters.upcomingTurretAngle(), DELTA);
  }

  @Test
  void zeroDistanceProducesFiniteParameters() {
    var robotPose =
        new Pose2d(
            FieldUtil.HUB_POSE
                .getTranslation()
                .minus(TurretConfig.TURRET_TO_ROBOT.getTranslation()),
            Rotation2d.kZero);
    var parameters =
        AimParameterUtil.getScoringParameters(robotPose, new ChassisSpeeds(0.0, 0.0, 1.0));

    assertThat(Double.isFinite(parameters.turretAngle())).isTrue();
    assertThat(Double.isFinite(parameters.distance())).isTrue();
    assertThat(Double.isFinite(parameters.turretTolerance())).isTrue();
    assertThat(Double.isFinite(parameters.turretFeedForwardRadians())).isTrue();
    assertThat(Double.isFinite(parameters.upcomingTurretAngle())).isTrue();
  }
}
