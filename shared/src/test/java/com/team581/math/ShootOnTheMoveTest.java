package com.team581.math;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class ShootOnTheMoveTest {
  private static final double EPSILON = 1e-10;

  @Test
  void compensatedShotLandsAtGoalForArbitraryDirectionsAndDrag() {
    var solver = new ShootOnTheMove(distance -> 1.25, "Test");
    var origin = new Translation2d(1.2, -0.7);
    var goal = new Translation2d(7.0, 3.0);
    for (double drag : new double[] {0.0, 0.465}) {
      for (double vx : new double[] {-4.0, 0.0, 4.0}) {
        for (double vy : new double[] {-3.0, 0.0, 3.0}) {
          var solution = solver.solve(origin, goal, new ChassisSpeeds(vx, vy, 0.0), drag);
          assertThat(solution.valid()).isTrue();
          var landed =
              solution
                  .goal()
                  .plus(new Translation2d(vx, vy).times(solution.effectiveTimeOfFlight()));
          assertEquals(goal.getX(), landed.getX(), 1e-9);
          assertEquals(goal.getY(), landed.getY(), 1e-9);
        }
      }
    }
  }

  @Test
  void constantCommandPredictionIsIndependentOfTimePartition() {
    double first = ShootOnTheMove.predictVelocity(0.0, 3.0, 0.02, 0.15);
    double second = ShootOnTheMove.predictVelocity(first, 3.0, 0.024, 0.15);
    assertEquals(ShootOnTheMove.predictVelocity(0.0, 3.0, 0.044, 0.15), second, EPSILON);
    assertThat(ShootOnTheMove.predictVelocity(0.0, 3.0, 0.024, 0.15)).isGreaterThan(first);
    assertEquals(3.0, ShootOnTheMove.predictVelocity(3.0, 3.0, 0.024, 0.15), EPSILON);
  }

  @Test
  void distanceDependentTofMatchesClosedFormRadialSolution() {
    var solver = new ShootOnTheMove(distance -> 0.5 + 0.1 * distance, "Test");
    var solution =
        solver.solve(
            Translation2d.kZero,
            new Translation2d(5.0, 0.0),
            new ChassisSpeeds(2.0, 0.0, 0.0),
            0.0);
    assertThat(solution.valid()).isTrue();
    assertEquals((5.0 - 2.0 * 0.5) / (1 + 2.0 * 0.1), solution.goal().getX(), 1e-9);
    assertEquals(0.5 + 0.1 * solution.goal().getNorm(), solution.timeOfFlight(), 1e-9);
  }

  @Test
  void effectiveFlightTimeMatchesIntegratedDragAndZeroDragLimit() {
    assertEquals(1.3, ShootOnTheMove.effectiveTimeOfFlight(1.3, 0.0), EPSILON);
    assertEquals(1.3, ShootOnTheMove.effectiveTimeOfFlight(1.3, 1e-12), EPSILON);
    double integral = 0.0;
    double dt = 1.3 / 10000;
    for (int i = 0; i < 10000; i++) {
      integral += Math.exp(-0.465 * (i + 0.5) * dt) * dt;
    }
    assertEquals(integral, ShootOnTheMove.effectiveTimeOfFlight(1.3, 0.465), 1e-9);
    assertThat(Double.isNaN(ShootOnTheMove.effectiveTimeOfFlight(1.0, -1.0))).isTrue();
  }

  @Test
  void invalidAndNonConvergingTablesAreRejected() {
    var empty = new ShootOnTheMove(new InterpolatingDoubleTreeMap());
    assertThat(
            empty
                .solve(Translation2d.kZero, new Translation2d(4.0, 0.0), new ChassisSpeeds(), 0.0)
                .valid())
        .isFalse();
    var negative = new ShootOnTheMove(distance -> -1.0, "Test");
    assertThat(
            negative
                .solve(Translation2d.kZero, new Translation2d(4.0, 0.0), new ChassisSpeeds(), 0.0)
                .valid())
        .isFalse();
    var oscillating = new ShootOnTheMove(distance -> distance, "Test");
    assertThat(
            oscillating
                .solve(
                    Translation2d.kZero,
                    new Translation2d(4.0, 0.0),
                    new ChassisSpeeds(1.0, 0.0, 0.0),
                    0.0)
                .valid())
        .isFalse();
  }

  @Test
  void invalidSettingsDisablePrediction() {
    for (double invalid : new double[] {0.0, -1.0, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertEquals(2.0, ShootOnTheMove.predictVelocity(2.0, 4.0, invalid, 0.15));
      assertEquals(2.0, ShootOnTheMove.predictVelocity(2.0, 4.0, 0.02, invalid));
    }
    assertEquals(2.0, ShootOnTheMove.predictVelocity(2.0, Double.NaN, 0.02, 0.15));
  }

  @Test
  void reversalBrakesBeforeCrossingZero() {
    assertThat(ShootOnTheMove.predictVelocity(2.0, -2.0, 0.02, 0.15)).isGreaterThan(0.0);
    assertEquals(0.0, ShootOnTheMove.predictVelocity(2.0, -2.0, 0.15 * Math.log(2), 0.15), EPSILON);
    assertThat(ShootOnTheMove.predictVelocity(2.0, -2.0, 0.2, 0.15)).isLessThan(0.0);
    assertEquals(-2.0, ShootOnTheMove.predictVelocity(2.0, -2.0, 100.0, 0.15), EPSILON);
  }

  @Test
  void solverIsInvariantUnderFieldRotationAndTranslation() {
    var solver = new ShootOnTheMove(distance -> 0.8 + distance * 0.04, "Test");
    var origin = new Translation2d(1.0, 2.0);
    var goal = new Translation2d(6.0, 4.0);
    var velocity = new Translation2d(1.2, -0.6);
    var rotation = Rotation2d.fromDegrees(127);
    var offset = new Translation2d(-2.0, 3.0);
    var first =
        solver.solve(origin, goal, new ChassisSpeeds(velocity.getX(), velocity.getY(), 0.0), 0.465);
    var rotatedVelocity = velocity.rotateBy(rotation);
    var second =
        solver.solve(
            origin.rotateBy(rotation).plus(offset),
            goal.rotateBy(rotation).plus(offset),
            new ChassisSpeeds(rotatedVelocity.getX(), rotatedVelocity.getY(), 0.0),
            0.465);
    assertThat(first.valid() && second.valid()).isTrue();
    assertEquals(
        0.0, first.goal().rotateBy(rotation).plus(offset).getDistance(second.goal()), 1e-9);
    assertEquals(first.timeOfFlight(), second.timeOfFlight(), 1e-9);
  }

  @Test
  void startsAndChangesSpeedTowardCommandInEitherDirection() {
    // After one time constant, 1/e of the original velocity error remains.
    for (double sign : new double[] {-1.0, 1.0}) {
      for (double measured : new double[] {0.0, 1.0, 4.0}) {
        double predicted = ShootOnTheMove.predictVelocity(sign * measured, sign * 3.0, 0.15, 0.15);
        assertEquals(sign * (3.0 + (measured - 3.0) / Math.E), predicted, EPSILON);
      }
    }
  }
}
