package frc.robot.deploy;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Open;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class DeployControlTest {
  private static DifferentialMechanism<TalonFX> mechanism;

  @AfterAll
  static void cleanup() {
    DriverStationSim.resetData();
    DriverStationSim.notifyNewData();
    mechanism.getLeader().close();
    mechanism.getFollower().close();
  }

  @BeforeAll
  static void setupHardware() {
    HAL.initialize(500, 0);
    mechanism =
        new DifferentialMechanism<>(
            TalonFX::new,
            new DifferentialMotorConstants<TalonFXConfiguration>()
                .withLeaderId(55)
                .withFollowerId(56)
                .withCANBusName("sim")
                .withAlignment(MotorAlignmentValue.Opposed)
                .withLeaderInitialConfigs(DeployConfig.LEFT_MOTOR_CONFIG)
                .withFollowerInitialConfigs(DeployConfig.RIGHT_MOTOR_CONFIG)
                .withFollowerUsesCommonLeaderConfigs(true));
  }

  private Deploy deploy;

  private void setAngles(double angle) throws Exception {
    setInput("leftMotorPosition", angle);
    setInput("rightMotorPosition", angle);
    setInput("differentialMechanismPosition", angle);
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
      assertThat(mechanism.getLeader().getAppliedControl()).isInstanceOf(NeutralOut.class);
      assertThat(mechanism.getFollower().getAppliedControl()).isInstanceOf(NeutralOut.class);

      SimHooks.stepTimingAsync(0.49);
      for (var request :
          new Runnable[] {
            deploy::intakeRequest,
            deploy::stowRequest,
            deploy::hopperCompactionRequest,
            deploy::waitHopperCompactionRequest,
            deploy::feedCompactionRequest,
            deploy::beastModeRequest,
            deploy::fixDifferentialDesyncRequest
          }) {
        request.run();
        deploy.robotPeriodic();
        assertThat(deploy.getState()).isEqualTo(DeployState.AUTO_INTAKE_DELAY);
      }
      SimHooks.stepTimingAsync(0.02);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE);
      assertThat(mechanism.getLeader().getAppliedControl())
          .isInstanceOf(Diff_MotionMagicTorqueCurrentFOC_Position.class);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void controlTargetsUseMechanismRotations() {
    deploy.stowRequest();
    var request =
        (Diff_MotionMagicTorqueCurrentFOC_Position) mechanism.getLeader().getAppliedControl();
    assertThat(request.AverageRequest.Position)
        .isEqualTo(-DeployState.STOW.getAngleDegrees() / 360.0);
    assertThat(DeployConfig.LEFT_MOTOR_CONFIG.Feedback.SensorToMechanismRatio).isEqualTo(36.57);
  }

  @Test
  void desyncOverrideUsesDifferenceSlot2AndRestoresNormalSlotsOnRelease() {
    deploy.fixDifferentialDesyncRequest();
    for (int i = 0; i < 2; i++) {
      var request =
          (Diff_PositionTorqueCurrentFOC_Position) mechanism.getLeader().getAppliedControl();
      assertThat(request.AverageRequest.Slot).isEqualTo(0);
      assertThat(request.AverageRequest.Position)
          .isEqualTo(-DeployState.FIX_DIFFERENTIAL_DESYNC.getAngleDegrees() / 360.0);
      assertThat(request.DifferentialRequest.Slot).isEqualTo(2);
      assertThat(request.DifferentialRequest.Position).isEqualTo(0);
      deploy.robotPeriodic();
    }

    deploy.stopFixDifferentialDesyncRequest();
    var request =
        (Diff_MotionMagicTorqueCurrentFOC_Position) mechanism.getLeader().getAppliedControl();
    assertThat(request.AverageRequest.Slot).isEqualTo(0);
    assertThat(request.DifferentialRequest.Slot).isEqualTo(1);
  }

  @Test
  void disabledHomingKeepsVoltageRequestAndWaitsForEnable() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    deploy.homeInAutoRequest();
    deploy.robotPeriodic();
    assertThat(deploy.getState()).isEqualTo(DeployState.HOME_INWARD);
    var request = (Diff_VoltageOut_Open) mechanism.getLeader().getAppliedControl();
    assertThat(request.AverageRequest.Output).isEqualTo(DeployConfig.HOMING_VOLTAGE_INWARD);

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
    deploy = new Deploy(mechanism);
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
        var request =
            (Diff_MotionMagicTorqueCurrentFOC_Position) mechanism.getLeader().getAppliedControl();
        assertThat(request.AverageRequest.Position).isEqualTo(-DeployConfig.MAX_ANGLE / 360.0);
        setAngles(DeployConfig.MAX_ANGLE);
        assertThat(deploy.atGoal()).isTrue();

        publisher.set(DeployConfig.MIN_ANGLE - 10);
        deploy.robotPeriodic();
        request =
            (Diff_MotionMagicTorqueCurrentFOC_Position) mechanism.getLeader().getAppliedControl();
        assertThat(request.AverageRequest.Position).isEqualTo(-DeployConfig.MIN_ANGLE / 360.0);
        setAngles(DeployConfig.MIN_ANGLE);
        assertThat(deploy.atGoal()).isTrue();
      } finally {
        publisher.set(originalAngle);
      }
    }
  }
}
