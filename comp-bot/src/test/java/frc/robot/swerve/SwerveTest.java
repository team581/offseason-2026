package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class SwerveTest {
  private static final double DELTA = 1e-9;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void calculatesFiveInchScoringXSwerveToleranceFromDistance() {
    var scoringDistanceMeters = 2.23;
    var expectedToleranceDegrees =
        Math.toDegrees(Math.atan2(Units.inchesToMeters(5.0), scoringDistanceMeters));
    var previousTenInchToleranceDegrees =
        Math.toDegrees(Math.atan2(Units.inchesToMeters(10.0), scoringDistanceMeters));

    assertEquals(
        expectedToleranceDegrees,
        Swerve.calculateScoringXSwerveToleranceDegrees(scoringDistanceMeters),
        DELTA);
    assertEquals(3.26, expectedToleranceDegrees, 0.01);
    assertEquals(6.50, previousTenInchToleranceDegrees, 0.01);
  }
}
