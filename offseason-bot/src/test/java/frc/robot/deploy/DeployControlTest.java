package frc.robot.deploy;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class DeployControlTest {
  private static TalonFX leftMotor;
  private static TalonFX rightMotor;

  @AfterAll
  static void cleanup() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    leftMotor.close();
    rightMotor.close();
  }

  @BeforeAll
  static void setupHardware() {
    HAL.initialize(500, 0);
    leftMotor = new TalonFX(55);
    rightMotor = new TalonFX(56);
  }

  private Deploy deploy;

  private void setAngles(double angle) throws Exception {
    setInput("leftMotorPosition", angle);
    setInput("rightMotorPosition", angle);
    setInput("averagePosition", angle);
  }

  private void setInput(String name, double value) throws Exception {
    var field = Deploy.class.getDeclaredField(name);
    field.setAccessible(true);
    field.setDouble(deploy, value);
  }

  @Test
  void autonomousHomingDelaysIntakeAndIgnoresHopperRequests() throws Exception {
    SimHooks.pauseTiming();
    try {
      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      deploy.homeInAutoRequest();
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
      assertThat(deploy.getState().isHoming()).isTrue();
      assertThat(deploy.atGoal()).isFalse();
      assertThat(leftMotor.getAppliedControl()).isInstanceOf(NeutralOut.class);
      assertThat(rightMotor.getAppliedControl()).isInstanceOf(NeutralOut.class);

      SimHooks.stepTimingAsync(0.49);
      for (var request :
          new Runnable[] {
            deploy::intakeRequest,
            deploy::stowRequest,
            deploy::hopperCompactionRequest,
            deploy::waitHopperCompactionRequest,
            deploy::feedCompactionRequest,
            deploy::beastModeRequest
          }) {
        request.run();
        deploy.robotPeriodic();
        assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
      }
      SimHooks.stepTimingAsync(0.02);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE);
      assertThat(leftMotor.getAppliedControl()).isInstanceOf(MotionMagicTorqueCurrentFOC.class);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void controlTargetsUseMechanismRotations() {
    deploy.stowRequest();
    var request = (MotionMagicTorqueCurrentFOC) leftMotor.getAppliedControl();
    assertThat(request.Position).isEqualTo(-DeployState.STOW.getAngleDegrees() / 360.0);
    assertThat(DeployConfig.LEFT_MOTOR_CONFIG.Feedback.SensorToMechanismRatio).isEqualTo(36.57);
  }

  @Test
  void disabledHomingKeepsVoltageRequestAndWaitsForEnable() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    deploy.homeInAutoRequest();
    deploy.robotPeriodic();
    assertThat(deploy.getState()).isEqualTo(DeployState.HOME_INWARD);
    var request = (VoltageOut) leftMotor.getAppliedControl();
    assertThat(request.Output).isEqualTo(DeployConfig.HOMING_VOLTAGE_INWARD);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    deploy.robotPeriodic();
    assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
  }

  @Test
  void disabledTimeDoesNotConsumeAutonomousIntakeDelay() {
    SimHooks.pauseTiming();
    try {
      DriverStationSim.setAutonomous(true);
      DriverStationSim.notifyNewData();
      deploy.homeInAutoRequest();
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      SimHooks.stepTimingAsync(2);
      deploy.robotPeriodic();
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
      SimHooks.stepTimingAsync(0.51);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @BeforeEach
  void setup() throws Exception {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
    deploy = new Deploy(leftMotor, rightMotor);
    deploy.robotPeriodic();
    deploy.homeInAutoRequest();
    setInput("leftStatorCurrent", 30);
    setInput("rightStatorCurrent", 30);
    setAngles(DeployConfig.MAX_ANGLE);
    deploy.robotPeriodic();
    assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE);
  }

  @Test
  void stowClampsTunableTargetOnEntryAndWhileInState() throws Exception {
    var field = DeployState.class.getDeclaredField("tunableAngle");
    field.setAccessible(true);
    var angle = (DoubleSubscriber) field.get(DeployState.STOW);
    double originalAngle = angle.get();
    try (var publisher = angle.getTopic().publish()) {
      try {
        publisher.set(DeployConfig.MAX_ANGLE + 10);
        deploy.stowRequest();
        var request = (MotionMagicTorqueCurrentFOC) leftMotor.getAppliedControl();
        assertThat(request.Position).isEqualTo(-DeployConfig.MAX_ANGLE / 360.0);
        setAngles(DeployConfig.MAX_ANGLE);
        assertThat(deploy.atGoal()).isTrue();

        publisher.set(DeployConfig.MIN_ANGLE - 10);
        deploy.robotPeriodic();
        request = (MotionMagicTorqueCurrentFOC) leftMotor.getAppliedControl();
        assertThat(request.Position).isEqualTo(-DeployConfig.MIN_ANGLE / 360.0);
        setAngles(DeployConfig.MIN_ANGLE);
        assertThat(deploy.atGoal()).isTrue();
      } finally {
        publisher.set(originalAngle);
      }
    }
  }
}
