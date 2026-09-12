package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

final class TurretConfigTest {
  @Test
  void motionMagicLimitsUseMechanismRotations() {
    assertThat(TurretConfig.MOTOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity)
        .isEqualTo(Units.radiansToRotations(10.0));
    assertThat(TurretConfig.MOTOR_CONFIG.MotionMagic.MotionMagicAcceleration)
        .isEqualTo(Units.radiansToRotations(35.0));
  }
}
