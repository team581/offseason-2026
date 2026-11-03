package frc.robot.robot_manager.hopper_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import frc.robot.Hardware;
import frc.robot.conveyor.Conveyor;
import frc.robot.conveyor.ConveyorState;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.FeederState;
import frc.robot.funneler.Funneler;
import frc.robot.funneler.FunnelerState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class HopperManagerConveyorCoverageTest {
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
  void ejectingCommandsConveyorEject() {
    hopperManager.setDriverWantsEject(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.EJECTING);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    // Verify whileInState maintains eject
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void feedAndIntakeCommandsConveyorFeed() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED_AND_INTAKE);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.FEED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);

    // Verify whileInState maintains feed
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.FEED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);
  }

  @Test
  void feedStartsInitialShotAndMaintainsFeedState() {
    hopperManager.feedRequest();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.INITIAL_SHOT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);

    // Verify whileInState maintains requests
    hopperManager.periodic();
    assertThat(conveyor.getState()).isIn(ConveyorState.INITIAL_SHOT, ConveyorState.FEED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);
  }

  @Test
  void idleDeployedCommandsConveyor() {
    hopperManager.idleRequest();
    hopperManager.periodic();
    // In IDLE_DEPLOYED without balls, conveyor should be IDLE
    assertThat(conveyor.getState()).isIn(ConveyorState.IDLE, ConveyorState.BALL_FILLING);
    assertThat(funneler.getState()).isIn(FunnelerState.IDLE, FunnelerState.BALL_FILLING);

    // Verify whileInState continues commanding
    hopperManager.periodic();
    assertThat(conveyor.getState()).isIn(ConveyorState.IDLE, ConveyorState.BALL_FILLING);
    assertThat(funneler.getState()).isIn(FunnelerState.IDLE, FunnelerState.BALL_FILLING);
  }

  @Test
  void idleStowedCommandsConveyorIdle() {
    hopperManager.setOperatorWantsStow(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_STOWED);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    // Verify whileInState maintains idle (Issue #131)
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void intakingCommandsConveyor() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.INTAKING);
    // In INTAKING, conveyor is either INTAKE, BALL_FILLING, or IDLE (if tower debounced)
    assertThat(conveyor.getState())
        .isIn(ConveyorState.INTAKE, ConveyorState.BALL_FILLING, ConveyorState.IDLE);
    assertThat(funneler.getState())
        .isIn(FunnelerState.INTAKE, FunnelerState.BALL_FILLING, FunnelerState.IDLE);

    // Verify whileInState continues updating
    hopperManager.periodic();
    assertThat(conveyor.getState())
        .isIn(ConveyorState.INTAKE, ConveyorState.BALL_FILLING, ConveyorState.IDLE);
    assertThat(funneler.getState())
        .isIn(FunnelerState.INTAKE, FunnelerState.BALL_FILLING, FunnelerState.IDLE);
  }

  @Test
  void scoreAndIntakeCommandsConveyorScore() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE_AND_INTAKE);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);

    // Verify whileInState maintains score
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);
  }

  @Test
  void scoreStartsInitialShotAndCompactionDelaysToScore() {
    hopperManager.scoreRequest();
    // Right after transition before timeout
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.INITIAL_SHOT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);

    // Verify whileInState maintains initial shot before timeout and keeps funneler in score
    hopperManager.periodic();
    assertThat(conveyor.getState()).isIn(ConveyorState.INITIAL_SHOT, ConveyorState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);
  }

  @Test
  void scoreWithOperatorStowCommandsConveyor() {
    hopperManager.setOperatorWantsStow(true);
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE);
    assertThat(conveyor.getState()).isIn(ConveyorState.INITIAL_SHOT, ConveyorState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);
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
  void transitionsValidateNoStaleRequests() {
    // Start in FEED -> funneler should be FEEDING
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);

    // Transition FEED -> SCORE: funneler updates from FEEDING to SCORE immediately
    hopperManager.scoreRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);

    // Transition SCORE -> FEED: funneler updates from SCORE to FEEDING immediately
    hopperManager.feedRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.FEED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.FEEDING);

    // Transition FEED -> IDLE: funneler updates to IDLE (or BALL_FILLING), not stuck in FEEDING
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(funneler.getState()).isIn(FunnelerState.IDLE, FunnelerState.BALL_FILLING);

    // Transition IDLE -> STOW: funneler updates to IDLE
    hopperManager.setOperatorWantsStow(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_STOWED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void unjammingEjectsIntakeAndConveyorWhileShootingThroughFeederAndFunneler() {
    hopperManager.unjamRequest();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);
    assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
    assertThat(feeder.getState()).isEqualTo(FeederState.SCORE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);
    assertThat(FunnelerState.SCORE.getVoltage()).isPositive();

    // Unjam must restore every transport request on subsequent cycles.
    intake.idleRequest();
    conveyor.idleRequest();
    feeder.idleRequest();
    funneler.idleRequest();
    for (int i = 0; i < 3; i++) {
      hopperManager.periodic();
      assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);
      assertThat(intake.getState()).isEqualTo(IntakeState.EJECT);
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
      assertThat(feeder.getState()).isEqualTo(FeederState.SCORE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.SCORE);
    }

    hopperManager.idleRequest();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    hopperManager.setDriverWantsIntake(true);
    hopperManager.unjamRequest();
    hopperManager.idleRequest();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.INTAKING);
    assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.INTAKE);
    assertThat(feeder.getState()).isEqualTo(FeederState.INTAKING);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.INTAKE);
  }
}
