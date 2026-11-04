package frc.robot.deploy;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Position;
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
  void bothMotorsCoastAndRepeatedIntakeRequestsDoNotRestorePid() throws Exception {
    setAngles(DeployState.INTAKE.getAngleDegrees() + 4);
    deploy.robotPeriodic();
    for (int i = 0; i < 5; i++) {
      deploy.intakeRequest();
      deploy.robotPeriodic();
      assertThat(mechanism.getLeader().getAppliedControl()).isInstanceOf(CoastOut.class);
      assertThat(mechanism.getFollower().getAppliedControl()).isInstanceOf(CoastOut.class);
    }
    assertThat(deploy.isFullyExtended()).isTrue();
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
  void failedRecoveryCoastsUntilAnotherCommand() throws Exception {
    SimHooks.pauseTiming();
    try {
      setAngles(0);
      deploy.robotPeriodic();
      setAngles(20);
      deploy.robotPeriodic();
      SimHooks.stepTimingAsync(3.1);
      deploy.robotPeriodic();
      SimHooks.stepTimingAsync(5.1);
      deploy.robotPeriodic();
      deploy.intakeRequest();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_RECOVERY_FAILED);
      assertThat(mechanism.getLeader().getAppliedControl()).isInstanceOf(CoastOut.class);
      deploy.stowRequest();
      assertThat(deploy.getState()).isEqualTo(DeployState.STOW);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void oneRaisedSideTriggersTheDelayAndExcessiveDesyncPreventsRecovery() throws Exception {
    SimHooks.pauseTiming();
    try {
      setAngles(0);
      deploy.robotPeriodic();
      setInput("leftMotorPosition", 20);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COLLAPSED);
      SimHooks.stepTimingAsync(3.1);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_RECOVERY_FAILED);
      assertThat(mechanism.getFollower().getAppliedControl()).isInstanceOf(CoastOut.class);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void recoveryWaitsContinuouslyThenUsesNormalPid() throws Exception {
    SimHooks.pauseTiming();
    try {
      setAngles(0);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COAST);
      setAngles(20);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COLLAPSED);
      SimHooks.stepTimingAsync(2);
      deploy.intakeRequest();
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COLLAPSED);
      setAngles(8);
      deploy.robotPeriodic();
      setAngles(20);
      deploy.robotPeriodic();
      SimHooks.stepTimingAsync(2);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COLLAPSED);
      SimHooks.stepTimingAsync(1.1);
      deploy.intakeRequest();
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_RECOVERING);
      assertThat(mechanism.getLeader().getAppliedControl())
          .isInstanceOf(Diff_MotionMagicTorqueCurrentFOC_Position.class);
      setAngles(4);
      deploy.robotPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.INTAKE_COAST);
      assertThat(mechanism.getFollower().getAppliedControl()).isInstanceOf(CoastOut.class);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @BeforeEach
  void setup() throws Exception {
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
  void stowAndCompactionImmediatelyOverrideCoast() throws Exception {
    for (var command : new Runnable[] {deploy::stowRequest, deploy::feedCompactionRequest}) {
      deploy.intakeRequest();
      setAngles(DeployState.INTAKE.getAngleDegrees());
      deploy.robotPeriodic();
      command.run();
      assertThat(mechanism.getLeader().getAppliedControl())
          .isInstanceOf(Diff_MotionMagicTorqueCurrentFOC_Position.class);
    }
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
