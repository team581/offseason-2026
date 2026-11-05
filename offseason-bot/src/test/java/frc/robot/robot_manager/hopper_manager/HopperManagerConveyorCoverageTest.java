package frc.robot.robot_manager.hopper_manager;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Hardware;
import frc.robot.conveyor.Conveyor;
import frc.robot.conveyor.ConveyorState;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.feeder.FeederState;
import frc.robot.funneler.Funneler;
import frc.robot.funneler.FunnelerState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class HopperManagerConveyorCoverageTest {
  private static Hardware hardware;

  private static FeederState expectedFeederState(HopperState state) {
    return switch (state) {
      case IDLE_DEPLOYED, IDLE_STOWED, INTAKING -> FeederState.IDLE;
      case EJECTING -> FeederState.EJECT;
      case UNJAMMING, SCORE, SCORE_AND_INTAKE -> FeederState.SCORE;
      case FEED, FEED_AND_INTAKE -> FeederState.FEED;
    };
  }

  private static FunnelerState expectedFunnelerState(HopperState state) {
    return switch (state) {
      case IDLE_DEPLOYED, IDLE_STOWED, INTAKING, EJECTING -> FunnelerState.IDLE;
      case UNJAMMING, SCORE, SCORE_AND_INTAKE -> FunnelerState.SCORE;
      case FEED, FEED_AND_INTAKE -> FunnelerState.FEEDING;
    };
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    hardware = new Hardware();
  }

  private Deploy deploy;
  private boolean deployFullyExtended;
  private Intake intake;
  private Conveyor conveyor;

  private Feeder feeder;

  private Funneler funneler;

  private HopperManager hopperManager;

  @Test
  void ballFillRunsConveyorAndFunnelerForTwoSecondsOncePerIntakeCycle() {
    SimHooks.pauseTiming();
    try {
      intake.periodic();
      hopperManager.periodic();
      hopperManager.setDriverWantsIntake(true);
      hopperManager.idleRequest();
      SimHooks.stepTimingAsync(3.1);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

      // The two-second window starts when the deploy is ready, not at intake entry.
      deployFullyExtended = true;
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.BALL_FILLING);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.BALL_FILLING);
      assertThat(FunnelerState.BALL_FILLING.getVoltage()).isEqualTo(1);
      assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);

      SimHooks.stepTimingAsync(1.99);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.BALL_FILLING);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.BALL_FILLING);
      assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);

      SimHooks.stepTimingAsync(0.02);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
      assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
      assertThat(intake.getState()).isEqualTo(IntakeState.INTAKE);

      SimHooks.stepTimingAsync(10.0);
      hopperManager.idleRequest();
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

      hopperManager.setDriverWantsIntake(false);
      hopperManager.idleRequest();
      hopperManager.setDriverWantsIntake(true);
      hopperManager.idleRequest();
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      SimHooks.stepTimingAsync(3.1);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.BALL_FILLING);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.BALL_FILLING);
      assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void ejectingKeepsConveyorAndFeederEjectingUntilCancelled() {
    hopperManager.setDriverWantsEject(true);
    hopperManager.idleRequest();
    assertThat(feeder.getState()).isEqualTo(FeederState.EJECT);
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.EJECTING);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
    assertThat(feeder.getState()).isEqualTo(FeederState.EJECT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    // Verify whileInState maintains eject
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.EJECT);
    assertThat(feeder.getState()).isEqualTo(FeederState.EJECT);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
    hopperManager.setDriverWantsEject(false);
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
  }

  @EnumSource(HopperState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceStaleFeederAndFunnelerRequests(HopperState state) {
    var expectedFeeder = expectedFeederState(state);
    var expectedFunneler = expectedFunnelerState(state);
    for (boolean entry : new boolean[] {true, false}) {
      if (expectedFeeder == FeederState.IDLE) {
        feeder.scoreRequest();
      } else {
        feeder.idleRequest();
      }
      if (expectedFunneler == FunnelerState.IDLE) {
        funneler.scoreRequest();
      } else {
        funneler.idleRequest();
      }
      if (entry) {
        hopperManager.afterTransition(state);
      } else {
        hopperManager.whileInState(state);
      }
      assertThat(feeder.getState()).as("%s entry=%s", state, entry).isEqualTo(expectedFeeder);
      assertThat(funneler.getState()).as("%s entry=%s", state, entry).isEqualTo(expectedFunneler);
    }
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
  void idleDeployedKeepsTransportIdle() {
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    // Verify whileInState continues commanding
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
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
  void intakingKeepsTransportIdle() {
    hopperManager.setDriverWantsIntake(true);
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.INTAKING);
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

    // Restore idle even if a previous request left the transport running.
    conveyor.scoreRequest();
    feeder.scoreRequest();
    funneler.scoreRequest();
    hopperManager.periodic();
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
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
    deployFullyExtended = false;
    deploy =
        new Deploy(hardware.deployDifferentialMechanism) {
          @Override
          public boolean isFullyExtended() {
            return deployFullyExtended;
          }
        };
    intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
    conveyor =
        new Conveyor(hardware.conveyorMotor, hardware.tunnelTopMotor, hardware.tunnelBottomMotor);
    feeder = new Feeder(hardware.feederMotor);
    funneler = new Funneler(hardware.funnelerMotor);

    hopperManager = new HopperManager(deploy, intake, conveyor, feeder, funneler);
  }

  @Test
  void stowingImmediatelyStopsBallFilling() {
    SimHooks.pauseTiming();
    try {
      intake.periodic();
      hopperManager.periodic();
      deployFullyExtended = true;
      hopperManager.setDriverWantsIntake(true);
      hopperManager.idleRequest();
      SimHooks.stepTimingAsync(2.9);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
      SimHooks.stepTimingAsync(0.2);
      hopperManager.periodic();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.BALL_FILLING);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.BALL_FILLING);

      hopperManager.setDriverWantsIntake(false);
      hopperManager.setOperatorWantsStow(true);
      hopperManager.idleRequest();
      assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
      assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
      assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void stowOperatorCommandsDeployStowAndIntakeIdleInScoreAndFeed() {
    deploy.homeInAutoRequest();
    deploy.simulationPeriodic();
    hopperManager.setOperatorWantsStow(true);

    for (boolean entry : new boolean[] {true, false}) {
      deploy.intakeRequest();
      intake.scoreRequest();
      if (entry) {
        hopperManager.afterTransition(HopperState.SCORE);
      } else {
        hopperManager.whileInState(HopperState.SCORE);
      }
      assertThat(deploy.getState()).isEqualTo(DeployState.STOW);
      assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);

      deploy.intakeRequest();
      intake.feedRequest();
      if (entry) {
        hopperManager.afterTransition(HopperState.FEED);
      } else {
        hopperManager.whileInState(HopperState.FEED);
      }
      assertThat(deploy.getState()).isEqualTo(DeployState.STOW);
      assertThat(intake.getState()).isEqualTo(IntakeState.IDLE);
    }
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

    // Transition FEED -> IDLE: funneler updates to IDLE
    hopperManager.idleRequest();
    hopperManager.periodic();
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);

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
    assertThat(FunnelerState.SCORE.getVoltage()).isEqualTo(10);

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
    assertThat(conveyor.getState()).isEqualTo(ConveyorState.IDLE);
    assertThat(feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void unjamReversesFloorAndTunnelWhileFeederRunsForwardThenIdleStopsAll() {
    hopperManager.unjamRequest();
    for (var motor :
        new TalonFX[] {
          hardware.conveyorMotor, hardware.tunnelTopMotor, hardware.tunnelBottomMotor
        }) {
      var request = (VoltageOut) motor.getAppliedControl();
      assertThat(request.Output).isNegative();
    }
    assertThat(((VoltageOut) hardware.feederMotor.getAppliedControl()).Output).isPositive();

    hopperManager.idleRequest();
    for (var motor :
        new TalonFX[] {
          hardware.conveyorMotor,
          hardware.tunnelTopMotor,
          hardware.tunnelBottomMotor,
          hardware.feederMotor
        }) {
      assertThat(motor.getAppliedControl()).isInstanceOf(NeutralOut.class);
    }
  }
}
