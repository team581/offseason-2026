package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

final class TurretCalculatorTest {
  @CsvSource({
    "-0.15, -0.9",
    "-0.499, -2.994",
    "-0.95, 0.3",
    "-12.15, -0.9",
    "0.0, 0.0",
    "0.15, 0.9",
    "0.499, 2.994",
    "0.95, -0.3",
    "12.15, 0.9"
  })
  @ParameterizedTest
  void homesToNearestMotorRevolution(double rotorDelta, double correctionDegrees) {
    double position =
        TurretCalculator.calculateHomedPosition(TurretConfig.MOTOR_ROTOR_CAL_OFFSET + rotorDelta);

    assertThat(Units.rotationsToDegrees(position))
        .isCloseTo(TurretConfig.HOMING_END_POSITION + correctionDegrees, within(1e-9));
  }
}
