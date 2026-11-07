package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import frc.robot.shooter.ShooterConfig;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.turret.TurretConfig;
import frc.robot.util.AimParameterUtil.AimingParameters;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Exercises the public aim paths, turret geometry, TOF solver, and real RPM/hood tables together.
 */
final class SwerveLagShotIntegrationTest {
  private static AimingParameters aim(
      boolean feeding, Pose2d pose, ChassisSpeeds measured, ChassisSpeeds commanded) {
    return feeding
        ? AimParameterUtil.getFeedingParameters(FeedLocation.RIGHT, pose, measured, commanded)
        : AimParameterUtil.getScoringParameters(pose, measured, commanded);
  }

  private static double angleDelta(AimingParameters a, AimingParameters b) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(a.turretAngle() - b.turretAngle())));
  }

  private static double hood(boolean feeding, double distance) {
    return (feeding ? ShooterHoodConfig.DISTANCE_TO_FEED : ShooterHoodConfig.DISTANCE_TO_SCORE)
        .get(distance);
  }

  private static Pose2d pose(boolean feeding, double distance) {
    var goal = feeding ? FeedLocation.RIGHT.getTranslation() : FieldUtil.HUB_POSE.getTranslation();
    return new Pose2d(
        goal.minus(TurretConfig.TURRET_TO_ROBOT.getTranslation())
            .minus(new Translation2d(distance, 0.0)),
        Rotation2d.kZero);
  }

  private static double rpm(boolean feeding, double distance) {
    return (feeding ? ShooterConfig.DISTANCE_TO_FEEDING_RPM : ShooterConfig.DISTANCE_TO_SCORE_RPM)
        .get(distance);
  }

  private static void setTau(DoublePublisher publisher, double tau) {
    publisher.set(tau);
    assertEquals(tau, AimParameterUtil.VELOCITY_TIME_CONSTANT_SECONDS.getAsDouble(), 1e-12);
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void tunedLagLeavesSteadyShotsUnchangedAndMakesSmallCorrectTransientCorrections() {
    double originalTau = AimParameterUtil.VELOCITY_TIME_CONSTANT_SECONDS.getAsDouble();
    double originalHorizon = ShootOnTheMove.PREDICTION_HORIZON_SECONDS.getAsDouble();
    assertEquals(0.30, originalTau, 1e-12);
    double maxAngle = 0.0;
    double maxDistance = 0.0;
    double maxRpm = 0.0;
    double maxHood = 0.0;
    int cases = 0;
    try (var tau = AimParameterUtil.VELOCITY_TIME_CONSTANT_SECONDS.getTopic().publish();
        var horizon = ShootOnTheMove.PREDICTION_HORIZON_SECONDS.getTopic().publish()) {
      try {
        horizon.set(0.02);
        for (boolean feeding : new boolean[] {false, true}) {
          for (double distance :
              feeding ? new double[] {6.5, 8.7, 12.0} : new double[] {2.82, 3.94, 5.5}) {
            var pose = pose(feeding, distance);
            for (double steady : new double[] {-1.0, 0.0, 1.0}) {
              var speeds = new ChassisSpeeds(0.0, steady, 0.0);
              setTau(tau, 0.15);
              var old = aim(feeding, pose, speeds, speeds);
              setTau(tau, 0.30);
              var tuned = aim(feeding, pose, speeds, speeds);
              assertThat(tuned.solutionValid()).isTrue();
              assertEquals(0.0, angleDelta(tuned, old), 1e-9);
              assertEquals(old.distance(), tuned.distance(), 1e-9);
              assertEquals(old.turretFeedForwardRadians(), tuned.turretFeedForwardRadians(), 1e-9);
            }
            // Starts, speed changes, braking and reversals, in both signs and on both axes.
            for (double[] maneuver :
                new double[][] {{0.0, 0.5}, {0.5, 1.0}, {1.0, 0.5}, {0.25, -0.25}}) {
              for (double sign : new double[] {-1.0, 1.0}) {
                for (boolean radial : new boolean[] {false, true}) {
                  double v = sign * maneuver[0];
                  double u = sign * maneuver[1];
                  var measured = new ChassisSpeeds(radial ? v : 0.0, radial ? 0.0 : v, 0.0);
                  var commanded = new ChassisSpeeds(radial ? u : 0.0, radial ? 0.0 : u, 0.0);
                  // Holding the measured speed disables the linear lag correction, including
                  // its acceleration contribution, without disabling normal motion compensation.
                  setTau(tau, 0.30);
                  var measuredOnly = aim(feeding, pose, measured, measured);
                  var tuned = aim(feeding, pose, measured, commanded);
                  setTau(tau, 0.15);
                  var old = aim(feeding, pose, measured, commanded);
                  assertThat(tuned.solutionValid()).isTrue();
                  assertThat(Double.isFinite(tuned.turretFeedForwardRadians())).isTrue();
                  double correction =
                      radial
                          ? tuned.distance() - measuredOnly.distance()
                          : angleDelta(tuned, measuredOnly);
                  // More velocity toward the goal shortens range; more +Y velocity aims toward -Y.
                  assertThat(correction * (u - v)).isNegative();
                  assertThat(Math.abs(correction)).isGreaterThan(1e-5);
                  if (!radial) {
                    assertThat(
                            (tuned.turretFeedForwardRadians()
                                    - measuredOnly.turretFeedForwardRadians())
                                * (u - v))
                        .isNegative();
                  }
                  double angleChange = Math.abs(angleDelta(tuned, old));
                  double distanceChange = Math.abs(tuned.distance() - old.distance());
                  double rpmChange =
                      Math.abs(rpm(feeding, tuned.distance()) - rpm(feeding, old.distance()));
                  double hoodChange =
                      Math.abs(hood(feeding, tuned.distance()) - hood(feeding, old.distance()));
                  assertThat(angleChange).isLessThan(1.0);
                  assertThat(distanceChange).isLessThan(0.04);
                  assertThat(rpmChange).isLessThan(20.0);
                  assertThat(hoodChange).isLessThan(0.25);
                  assertThat(Math.abs(angleDelta(tuned, measuredOnly))).isLessThan(1.0);
                  assertThat(Math.abs(tuned.distance() - measuredOnly.distance())).isLessThan(0.04);
                  maxAngle = Math.max(maxAngle, angleChange);
                  maxDistance = Math.max(maxDistance, distanceChange);
                  maxRpm = Math.max(maxRpm, rpmChange);
                  maxHood = Math.max(maxHood, hoodChange);
                  cases++;
                }
              }
            }
          }
        }
      } finally {
        setTau(tau, originalTau);
        horizon.set(originalHorizon);
      }
    }
    System.out.printf(
        "Lag shot integration: %d transients; max change vs 0.15 s: %.4f deg, %.4f m, %.2f RPM, %.4f hood deg%n",
        cases, maxAngle, maxDistance, maxRpm, maxHood);
  }
}
