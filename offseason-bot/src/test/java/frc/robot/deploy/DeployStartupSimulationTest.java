package frc.robot.deploy;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.HeadlessRunner;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

@Tag("headless")
final class DeployStartupSimulationTest {
  @Test
  @Timeout(30)
  void autoHomingWaitsForEnableThenHoldsUpBeforeIntaking() {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    try (var robot = new HeadlessRunner()) {
      double disabledUntil = Timer.getFPGATimestamp() + 1.0;
      while (Timer.getFPGATimestamp() < disabledUntil) {
        robot.tick();
        // The selected auto requests homing, but the deploy can't move while disabled.
        assertThat(robot.deploy().getState()).isEqualTo(DeployState.HOME_INWARD);
      }
      assertThat(robot.deploy().getAngleDegrees()).isGreaterThan(70.0);
      // Start away from the stop: enable alone must not declare it homed.
      robot.seedDeployAngle(45.0);
      robot.tick();
      DriverStationSim.setEnabled(true);
      robot.tick();
      assertThat(robot.deploy().getState()).isEqualTo(DeployState.HOME_INWARD);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.AUTO_INTAKE_DELAY, 3.0);
      double homedAt = Timer.getFPGATimestamp();
      assertThat(robot.deploy().getAngleDegrees()).isGreaterThan(70.0);
      while (Timer.getFPGATimestamp() - homedAt < 0.45) {
        robot.deploy().intakeRequest();
        robot.tick();
        assertThat(robot.deploy().getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
        assertThat(robot.deploy().getAngleDegrees()).isGreaterThan(70.0);
      }
      robot.runUntil(
          () -> robot.deploy().getState() == DeployState.INTAKE && robot.deploy().atGoal(), 3.0);
      assertThat(Timer.getFPGATimestamp() - homedAt).isGreaterThanOrEqualTo(0.48);
      assertThat(robot.deploy().getAngleDegrees()).isBetween(-5.0, 5.0);
    }
  }
}
