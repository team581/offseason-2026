package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/** Runs turret homing against Phoenix's simulated TalonFX firmware. */
final class TurretFirmwareHomingTest {
  // Raw integrated sensor fraction measured on the robot with the turret at center.
  private static final double RAW_ROTOR_AT_CENTER = -TurretConfig.MOTOR_ROTOR_OFFSET;

  private static int nextId = 40;

  private static double angle(TalonFX motor) {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  private static void assertHomesOnEveryEnableLoop(Turret turret) throws InterruptedException {
    // The first enabled loop can land on any disabled sample, so every one must be aligned.
    for (int i = 0; i < 25; i++) {
      Thread.sleep(20);
      turret.updateHoming(true);
      assertThat(turret.updateHoming(false)).as("Enable after disabled loop %d", i).isTrue();
    }
  }

  /** Constructs the turret, then applies the real robot's rotor offset that sim leaves out. */
  private static Turret newTurret(TalonFX motor) {
    var turret = new Turret(motor);
    assertThat(
            motor
                .getConfigurator()
                .apply(
                    new FeedbackConfigs()
                        .withSensorToMechanismRatio(TurretConfig.MOTOR_TO_TURRET)
                        .withFeedbackRotorOffset(TurretConfig.MOTOR_ROTOR_OFFSET))
                .isOK())
        .isTrue();
    return turret;
  }

  /** Steps homing at the robot loop rate long enough to pass the debounce several times. */
  private static void stepDisabled(Turret turret, double seconds) throws InterruptedException {
    for (int i = 0; i < seconds / 0.02; i++) {
      Thread.sleep(20);
      assertThat(turret.updateHoming(true)).isFalse();
    }
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.0, 0.0137, -0.31, 0.4})
  void homesToHandSetCenterAndSurvivesCodeRestarts(double rotorTurnsFromCenter)
      throws InterruptedException {
    var motor = new TalonFX(nextId++);
    var sim = motor.getSimState();
    sim.setSupplyVoltage(12);
    // A hand-set turret can sit any number of whole rotor turns from where the sensor booted.
    sim.setRawRotorPosition(7 + RAW_ROTOR_AT_CENTER + rotorTurnsFromCenter);
    double expected =
        TurretConfig.HOMING_END_POSITION
            + rotorTurnsFromCenter / TurretConfig.MOTOR_TO_TURRET * 360;

    var turret = newTurret(motor);
    stepDisabled(turret, 0.5);
    assertThat(angle(motor)).isCloseTo(expected, within(0.05));
    assertHomesOnEveryEnableLoop(turret);

    // Redeploying code keeps the device's position, so homing again must not move it.
    var restarted = newTurret(motor);
    stepDisabled(restarted, 0.5);
    assertThat(angle(motor)).isCloseTo(expected, within(0.05));
    assertHomesOnEveryEnableLoop(restarted);
    motor.close();
  }
}
