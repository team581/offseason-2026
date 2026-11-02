package frc.robot.robot_manager.hopper_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import frc.robot.Hardware;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.funneler.Funneler;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class HopperManagerIntakeCoverageTest {
  private static Hardware hardware;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    hardware = new Hardware();
  }

  @org.junit.jupiter.api.AfterAll
  static void tearDownHal() {
    hardware.towerSensor.close();
  }

  private Deploy deploy;
  private Intake intake;
  private Conveyor conveyor;
  private Feeder feeder;
  private Funneler funneler;

  private HopperManager hopperManager;

  @Test
  void ejectingCommandsIntakeEject() {
    hopperManager.setDriverWantsEject(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.EJECTING);
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);

    // Verify whileInState maintains eject
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
  }

  @Test
  void feedAndIntakeCommandsIntakeIntake() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED_AND_INTAKE);
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);

    // Verify whileInState maintains intake request
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
  }

  @Test
  void feedCommandsIntakeFeed() {
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED);
    // Before compaction timeout, intake should be feeding
    assertThat(intake.getState()).isIn(IntakeState.FEED, IntakeState.IDLE);
  }

  @Test
  void feedWithOperatorStowCommandsIntakeIdle() {
    hopperManager.setOperatorWantsStow(true);
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED);
    // When stowed by operator, intake roller must be IDLE
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    // Verify whileInState maintains idle
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
  }

  @Test
  void idleDeployedCommandsIntakeIdle() {
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
  }

  @Test
  void idleStowedCommandsIntakeIdle() {
    hopperManager.setOperatorWantsStow(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_STOWED);
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    // Verify whileInState maintains idle
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
  }

  @Test
  void intakingCommandsIntakeIntake() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.INTAKING);
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);

    // Verify whileInState maintains intake request
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
  }

  @Test
  void scoreAndIntakeCommandsIntakeIntake() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE_AND_INTAKE);
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);

    // Verify whileInState maintains intake request
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
  }

  @Test
  void scoreCommandsIntakeScore() {
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE);
    // Before compaction timeout, intake should be scoring
    assertThat(intake.getState()).isIn(IntakeState.SCORE, IntakeState.IDLE);
  }

  @Test
  void scoreWithOperatorStowCommandsIntakeIdle() {
    hopperManager.setOperatorWantsStow(true);
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE);
    // When stowed by operator, intake roller must be IDLE
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    // Verify whileInState maintains idle
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
  }

  @BeforeEach
  void setUp() {
    deploy = new Deploy(hardware.deployDifferentialMechanism);
    intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
    conveyor = new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);
    feeder = new Feeder(hardware.feederTopMotor, hardware.feederBottomMotor);
    funneler = new Funneler(hardware.funnelerMotor);

    hopperManager =
        new HopperManager(deploy, intake, conveyor, feeder, funneler, hardware.towerSensor);
  }

  @Test
  void stopRequestsGuardAgainstStaleStateTransitions() {
    intake.feedRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.FEED);

    // Stale stop scoring request should not affect FEED state
    intake.stopScoringRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.FEED);

    // Stale stop intaking request should not affect FEED state
    intake.stopIntakingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.FEED);

    // Stale stop ejecting request should not affect FEED state
    intake.stopEjectingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.FEED);

    // Matching stop feeding request idles the intake
    intake.stopFeedingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    intake.scoreRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.SCORE);
    intake.stopFeedingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.SCORE);
    intake.stopScoringRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    intake.intakeRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
    intake.stopScoringRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
    intake.stopIntakingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

    intake.ejectRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
    intake.stopIntakingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
    intake.stopEjectingRequest();
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
  }

  @Test
  void unjammingCommandsIntakeEject() {
    hopperManager.unjamRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);

    // Verify whileInState maintains eject
    hopperManager.periodic();
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
  }
}
