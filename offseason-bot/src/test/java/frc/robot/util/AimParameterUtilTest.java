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
  void accelerationFeedforwardIncludesMotionOfCompensatedGoal() {
    var solver = new ShootOnTheMove(distance -> 1.0, "AccelerationTest");
    var goal = new Translation2d(5.0, 2.0);
    var pose =
        new Pose2d(
            goal.minus(TurretConfig.TURRET_TO_ROBOT.getTranslation())
                .minus(new Translation2d(4.0, 0.0)),
            Rotation2d.kZero);
    double tau = ShootOnTheMove.VELOCITY_TIME_CONSTANT_SECONDS.getAsDouble();
    double h = ShootOnTheMove.PREDICTION_HORIZON_SECONDS.getAsDouble();
    double effectiveTof =
        ShootOnTheMove.effectiveTimeOfFlight(1.0, ShootOnTheMove.DRAG_CONSTANT.getAsDouble());
    for (double measured : new double[] {0.0, 0.5, 2.0}) {
      for (double commanded : new double[] {-2.0, 0.0, 3.0}) {
        var result =
            AimParameterUtil.calculateParameters(
                solver,
                goal,
                0.1,
                pose,
                new ChassisSpeeds(0.0, measured, 0.0),
                new ChassisSpeeds(0.0, commanded, 0.0),
                true);
        double v = commanded + (measured - commanded) * Math.exp(-h / tau);
        double a = (commanded - measured) / tau * Math.exp(-h / tau);
        double expected =
            4.0 * (-measured - a * effectiveTof) / (16.0 + v * v * effectiveTof * effectiveTof);
        assertEquals(expected, result.turretFeedForwardRadians(), 0.002);
      }
    }
  }

  @Test
  void constantTofFeedforwardMatchesAnalyticCompensatedBearingDerivative() {
    var solver = new ShootOnTheMove(distance -> 1.0, "AnalyticTest");
    var goal = new Translation2d(5.0, 2.0);
    var offset = TurretConfig.TURRET_TO_ROBOT.getTranslation();
    var pose = new Pose2d(goal.minus(offset).minus(new Translation2d(4.0, 0.0)), Rotation2d.kZero);
    double effectiveTof =
        ShootOnTheMove.effectiveTimeOfFlight(1.0, ShootOnTheMove.DRAG_CONSTANT.getAsDouble());
    for (double omega : new double[] {-1.0, 0.0, 1.0}) {
      var speeds = new ChassisSpeeds(0.0, 1.0, omega);
      var result =
          AimParameterUtil.calculateParameters(solver, goal, 0.1, pose, speeds, speeds, true);
      var vt = new Translation2d(-omega * offset.getY(), 1.0 + omega * offset.getX());
      var acceleration = offset.times(-omega * omega);
      var q = new Translation2d(4.0, 0.0).minus(vt.times(effectiveTof));
      var qDot = vt.unaryMinus().minus(acceleration.times(effectiveTof));
      double expected =
          (q.getX() * qDot.getY() - q.getY() * qDot.getX()) / (q.getNorm() * q.getNorm()) - omega;
      assertThat(result.solutionValid()).isTrue();
      assertEquals(expected, result.turretFeedForwardRadians(), 1e-4);
    }
  }

  @Test
  void dynamicSolutionAndLookaheadUseProjectedRobotPose() {
    var robotPose = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(25.0));
    var speeds = new ChassisSpeeds(0.7, -0.4, 0.6);
    var commandedSpeeds = speeds;
    var dynamic = AimParameterUtil.getScoringParameters(robotPose, speeds, commandedSpeeds);
    var stationary =
        AimParameterUtil.getStaticScoringParameters(robotPose, speeds, commandedSpeeds);

    assertNotEquals(stationary.turretAngle(), dynamic.turretAngle(), 1e-5);

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

    var future = AimParameterUtil.getScoringParameters(futureRobotPose, speeds, speeds);
    assertEquals(future.turretAngle(), dynamic.upcomingTurretAngle(), DELTA);
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
  void invalidFlightTableCannotProduceReadySolution() {
    var solver = new ShootOnTheMove(distance -> Double.NaN, "InvalidTest");
    var parameters =
        AimParameterUtil.calculateParameters(
            solver,
            new Translation2d(5.0, 0.0),
            0.1,
            Pose2d.kZero,
            new ChassisSpeeds(),
            new ChassisSpeeds(),
            true);
    assertThat(parameters.solutionValid()).isFalse();
    assertEquals(0.0, parameters.turretFeedForwardRadians());

    var nonFiniteCommand =
        AimParameterUtil.getScoringParameters(
            new Pose2d(2.0, 3.0, Rotation2d.k180deg),
            new ChassisSpeeds(1.0, -0.5, 0.2),
            new ChassisSpeeds(Double.NaN, Double.POSITIVE_INFINITY, Double.NaN));
    assertThat(nonFiniteCommand.solutionValid()).isTrue();
    assertThat(Double.isFinite(nonFiniteCommand.turretFeedForwardRadians())).isTrue();
    assertThat(Double.isFinite(nonFiniteCommand.upcomingTurretAngle())).isTrue();
  }

  @Test
  void lateralAndRotationalMotionProduceNegativeTurretFeedforward() {
    var offset = TurretConfig.TURRET_TO_ROBOT.getTranslation();
    var goal = FieldUtil.HUB_POSE.getTranslation();
    var robotPose =
        new Pose2d(goal.minus(offset).minus(new Translation2d(4.0, 0.0)), Rotation2d.kZero);
    var lateralSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    var lateral = AimParameterUtil.getScoringParameters(robotPose, lateralSpeeds, lateralSpeeds);
    var rotatingSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    var rotating = AimParameterUtil.getScoringParameters(robotPose, rotatingSpeeds, rotatingSpeeds);

    assertThat(lateral.turretFeedForwardRadians()).isNegative();
    assertThat(rotating.turretFeedForwardRadians()).isNegative();

    var staticLateral =
        AimParameterUtil.getStaticScoringParameters(robotPose, lateralSpeeds, lateralSpeeds);
    var staticRotating =
        AimParameterUtil.getStaticScoringParameters(robotPose, rotatingSpeeds, rotatingSpeeds);
    assertEquals(-1.0 / 4.0, staticLateral.turretFeedForwardRadians(), 1e-4);
    var rotatingTurretSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            rotatingSpeeds, robotPose.getRotation().getDegrees());
    assertEquals(
        -(1.0 + rotatingTurretSpeeds.vyMetersPerSecond / 4.0),
        staticRotating.turretFeedForwardRadians(),
        1e-4);
  }

  @Test
  void realTofTablesConvergeAcrossOperatingVelocities() {
    for (var solver :
        new ShootOnTheMove[] {AimParameterUtil.SCORING_SOTM, AimParameterUtil.FEEDING_SOTM}) {
      for (double distance : new double[] {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 9.0, 13.0}) {
        for (double vx : new double[] {-4.75, 0.0, 4.75}) {
          for (double vy : new double[] {-4.75, 0.0, 4.75}) {
            var result =
                solver.solve(
                    Translation2d.kZero,
                    new Translation2d(distance, 0.0),
                    new ChassisSpeeds(vx, vy, 0.0),
                    ShootOnTheMove.DRAG_CONSTANT.getAsDouble());
            assertThat(result.valid()).as("distance=%s, vx=%s, vy=%s", distance, vx, vy).isTrue();
            assertThat(result.residualMeters()).isLessThanOrEqualTo(1e-10);
          }
        }
      }
    }
  }

  @Test
  void staticFeedingUsesTwentyInchTolerance() {
    var robotPose = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(-20.0));
    var parameters =
        AimParameterUtil.getStaticFeedingParameters(
            FeedLocation.RIGHT, robotPose, new ChassisSpeeds(), new ChassisSpeeds());

    assertEquals(
        TurretCalculator.getGoalCentricTurretTolerance(
            FeedLocation.RIGHT.getTranslation(), robotPose, Units.inchesToMeters(20.0)),
        parameters.turretTolerance(),
        DELTA);
  }

  @Test
  void staticScoringUsesCornerTurretPoseAndFiveInchTolerance() {
    var robotPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(35.0));
    var parameters =
        AimParameterUtil.getStaticScoringParameters(
            robotPose, new ChassisSpeeds(), new ChassisSpeeds());
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
    var speeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    var parameters = AimParameterUtil.getScoringParameters(robotPose, speeds, speeds);

    assertThat(Double.isFinite(parameters.turretAngle())).isTrue();
    assertThat(Double.isFinite(parameters.distance())).isTrue();
    assertThat(Double.isFinite(parameters.turretTolerance())).isTrue();
    assertThat(Double.isFinite(parameters.turretFeedForwardRadians())).isTrue();
    assertThat(Double.isFinite(parameters.upcomingTurretAngle())).isTrue();
  }
}
