package frc.robot.robot_manager.hopper_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.config.RobotKind;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.funneler.Funneler;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public enum HopperBallPosition {
    CLOSE_TO_SHOOTER,
    AT_SENSOR,
    BELOW_SENSOR,
  }

  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;

  public final Feeder feeder;
  public final Funneler funneler;

  public final DigitalInput towerSensor;
  private final Debouncer towerSensorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;
  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;

  private boolean towerSensorRaw = false;

  private boolean shouldBeastMode = false;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  public HopperManager(
      Deploy deploy,
      Intake intake,
      Conveyor conveyor,
      Feeder feeder,
      Funneler funneler,
      DigitalInput towerSensor) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.funneler = funneler;
    this.towerSensor = towerSensor;
  }

  public void feedRequest() {
    setState(resolveFeedState());
  }

  public double getFeederToShooterTime() {
    return switch (getShotPosition()) {
      // TODO: Validate this
      case CLOSE_TO_SHOOTER -> 0.25;
      case AT_SENSOR -> 0.2;
      case BELOW_SENSOR -> 0.3;
    };
  }

  public boolean hasFinishedShooting() {
    if (getState() != HopperState.SCORE
        && getState() != HopperState.SCORE_AND_INTAKE
        && getState() != HopperState.FEED
        && getState() != HopperState.FEED_AND_INTAKE) {
      return false;
    }

    return !isShootingStrict();
  }

  public void idleRequest() {
    setState(resolveIdleState());
  }

  public boolean isFull() {
    if (RobotBase.isSimulation()) {
      return timeout(10.0);
    }
    return hopperCapacity == HopperCapacity.HIGH;
  }

  public boolean isIntaking() {
    return intake.getState() == IntakeState.INTAKE;
  }

  public boolean isShooting() {
    if (getState() != HopperState.SCORE
        && getState() != HopperState.SCORE_AND_INTAKE
        && getState() != HopperState.FEED
        && getState() != HopperState.FEED_AND_INTAKE) {
      return false;
    }

    return isShootingStrict();
  }

  public void scoreRequest() {
    scoreRequest(false);
  }

  public void scoreRequest(boolean shouldBeastMode) {
    this.shouldBeastMode = shouldBeastMode;
    setState(resolveScoreState());
  }

  public void setDriverWantsEject(boolean wantsEject) {
    driverWantsEject = wantsEject;
  }

  public void setDriverWantsIntake(boolean wantsIntake) {
    driverWantsIntake = wantsIntake;
  }

  public void setOperatorWantsStow(boolean wantsStow) {
    operatorWantsStow = wantsStow;
  }

  public void unjamRequest() {
    setState(HopperState.UNJAMMING);
  }

  private HopperBallPosition getShotPosition() {
    if (towerSensorDebounced) {
      return HopperBallPosition.AT_SENSOR;
    }

    if (shouldFillBalls()) {
      return HopperBallPosition.CLOSE_TO_SHOOTER;
    }

    return HopperBallPosition.BELOW_SENSOR;
  }

  private boolean isShootingStrict() {
    if (RobotBase.isSimulation()) {
      return !timeout(1.5);
    }
    return towerSensorDebounced;
  }

  private HopperState resolveFeedState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.FEED_AND_INTAKE;
    }

    return HopperState.FEED;
  }

  private HopperState resolveIdleState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.INTAKING;
    }

    if (operatorWantsStow) {
      return HopperState.IDLE_STOWED;
    }

    return HopperState.IDLE_DEPLOYED;
  }

  private HopperState resolveScoreState() {
    if (driverWantsEject) {
      return HopperState.EJECTING;
    }

    if (driverWantsIntake) {
      return HopperState.SCORE_AND_INTAKE;
    }

    return HopperState.SCORE;
  }

  private void setState(HopperState newState) {
    setStateFromRequest(newState);
  }

  private boolean shouldFillBalls() {
    if (towerSensorDebounced) {
      // The sensor in the tower shows we are holding fuel, so we can't fill anymore
      return false;
    }
    if (!deploy.isFullyExtended()) {
      return false;
    }

    return intake.hasBeenIntaking();
  }

  /**
   * Sets conveyor, feeder, and funneler to ball filling if conditions are met, otherwise idles
   * them.
   */
  private void smartBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
      funneler.ballFillingRequest();
    } else {
      conveyor.idleRequest();
      feeder.idleRequest();
      funneler.idleRequest();
    }
  }

  private void smartIntakeBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
      funneler.ballFillingRequest();
    } else {
      if (towerSensorDebounced) {
        conveyor.idleRequest();
        feeder.idleRequest();
        funneler.idleRequest();
      } else {
        conveyor.intakeRequest();
        feeder.intakeRequest();
        funneler.intakeRequest();
      }
    }
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE_DEPLOYED -> {
        intake.idleRequest();
        deploy.intakeRequest();
        smartBallFillRequest();
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
        funneler.idleRequest();
      }
      case INTAKING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        smartIntakeBallFillRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.ejectRequest();
        funneler.idleRequest();
      }
      case UNJAMMING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.scoreRequest();
        funneler.scoreRequest();
      }
      case SCORE -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (!operatorWantsStow
            && deploy.getState() != DeployState.SCORE_COMPACTION
            && deploy.getState() != DeployState.SCORE_COMPACTION_WAITING) {
          deploy.intakeRequest();
        }
        if (operatorWantsStow) {
          intake.idleRequest();
        } else {
          intake.scoreRequest();
        }
        conveyor.initialShotRequest();
        feeder.scoreRequest();
        funneler.scoreRequest();
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.scoreRequest();
        feeder.scoreRequest();
        funneler.scoreRequest();
      }

      case FEED -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (!operatorWantsStow && deploy.getState() != DeployState.FEED_COMPACTION) {
          deploy.intakeRequest();
        }
        intake.feedRequest();
        conveyor.initialShotRequest();
        feeder.feedRequest();
        funneler.feedingRequest();
      }
      case FEED_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.feedRequest();
        feeder.feedRequest();
        funneler.feedingRequest();
      }
    }
  }

  @Override
  protected void collectInputs() {
    if (RobotBase.isSimulation()) {
      towerSensorRaw =
          switch (getState()) {
            case IDLE_DEPLOYED, IDLE_STOWED, INTAKING -> timeout(5);
            default -> false;
          };
    } else if (DSOptions.USE_TOWER_SENSOR.getAsBoolean()) {
      towerSensorRaw = (RobotKind.IS_COMP_BOT != towerSensor.get());
    } else {
      towerSensorRaw = true;
    }
    towerSensorDebounced = towerSensorDebouncer.calculate(towerSensorRaw);

    hopperCapacity = intake.hasBeenIntaking() ? HopperCapacity.HIGH : HopperCapacity.LOW;
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (getState()) {
      case IDLE_DEPLOYED, IDLE_STOWED, INTAKING, EJECTING -> {
        yield resolveIdleState();
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(HopperState state) {
    if (state.canBallFill) {
      if (state == HopperState.INTAKING) {
        smartIntakeBallFillRequest();
      } else {
        smartBallFillRequest();
      }
    }

    switch (state) {
      case IDLE_DEPLOYED -> {
        // Conveyor, feeder, and funneler are updated above via smartBallFillRequest()
        deploy.intakeRequest();
        intake.idleRequest();
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
        funneler.idleRequest();
      }
      case INTAKING -> {
        // Conveyor, feeder, and funneler are updated above via smartIntakeBallFillRequest()
        deploy.intakeRequest();
        intake.intakeRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.idleRequest();
        funneler.idleRequest();
      }
      case UNJAMMING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.scoreRequest();
        funneler.scoreRequest();
      }
      case SCORE -> {
        if (operatorWantsStow) {
          deploy.stowRequest();
          intake.idleRequest();
          if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
            conveyor.scoreRequest();
          } else {
            conveyor.initialShotRequest();
          }
          feeder.scoreRequest();
          funneler.scoreRequest();
        } else if (shouldBeastMode) {
          deploy.beastModeRequest();
          intake.scoreRequest();
          conveyor.scoreRequest();
          feeder.scoreRequest();
          funneler.scoreRequest();
        } else if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          double shuffleInterval =
              HopperManagerConfig.HOPPER_COMPACTION_SHUFFLE_INTERVAL.getAsDouble();
          if (((int) (Timer.getFPGATimestamp() / shuffleInterval)) % 2 == 0) {
            deploy.hopperCompactionRequest();
          } else {
            deploy.intakeRequest();
          }
          intake.idleRequest();
          conveyor.scoreRequest();
          feeder.scoreRequest();
          funneler.scoreRequest();
        } else {
          deploy.waitHopperCompactionRequest();
          intake.scoreRequest();
          conveyor.initialShotRequest();
          feeder.scoreRequest();
          funneler.scoreRequest();
        }
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.scoreRequest();
        feeder.scoreRequest();
        funneler.scoreRequest();
      }
      case FEED -> {
        if (operatorWantsStow) {
          deploy.stowRequest();
          intake.idleRequest();
          if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
            conveyor.feedRequest();
          } else {
            conveyor.initialShotRequest();
          }
        } else if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.feedCompactionRequest();
          intake.idleRequest();
          conveyor.feedRequest();
        } else {
          deploy.intakeRequest();
          intake.feedRequest();
          conveyor.initialShotRequest();
        }
        feeder.feedRequest();
        funneler.feedingRequest();
      }
      case FEED_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.feedRequest();
        feeder.feedRequest();
        funneler.feedingRequest();
      }
    }

    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
    DogLog.forceNt.log("HopperManager/TowerSensor", towerSensorRaw);
    DogLog.log("HopperManager/IsShooting", isShooting());
  }
}
