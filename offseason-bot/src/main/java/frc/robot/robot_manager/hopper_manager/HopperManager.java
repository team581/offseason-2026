package frc.robot.robot_manager.hopper_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.funneler.Funneler;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  private static final double BALL_FILL_DURATION_SECONDS = 2.0;
  private final Timer ballFillTimer = new Timer();
  private boolean ballFillAvailable = true;
  private boolean ballFillActive = false;

  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;

  public final Feeder feeder;
  public final Funneler funneler;

  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;

  private boolean shouldBeastMode = false;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  public HopperManager(
      Deploy deploy, Intake intake, Conveyor conveyor, Feeder feeder, Funneler funneler) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.funneler = funneler;
  }

  public void feedRequest() {
    setState(resolveFeedState());
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

  private boolean isShootingStrict() {
    if (RobotBase.isSimulation()) {
      return !timeout(1.5);
    }
    return !feeder.hasFinishedShooting();
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

  private void updateBallFill() {
    if (ballFillAvailable
        && getState() == HopperState.INTAKING
        && deploy.isFullyExtended()
        && intake.hasBeenIntaking()) {
      ballFillAvailable = false;
      ballFillActive = true;
      ballFillTimer.restart();
    }

    if (ballFillActive && ballFillTimer.hasElapsed(BALL_FILL_DURATION_SECONDS)) {
      ballFillActive = false;
      ballFillTimer.stop();
    }

    if (ballFillActive) {
      conveyor.ballFillingRequest();
      funneler.ballFillingRequest();
    } else {
      conveyor.idleRequest();
      funneler.idleRequest();
    }
    feeder.idleRequest();
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE_DEPLOYED -> {
        intake.idleRequest();
        deploy.intakeRequest();
        updateBallFill();
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
        updateBallFill();
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
        if (operatorWantsStow) {
          deploy.stowRequest();
          intake.idleRequest();
        } else {
          // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
          if (deploy.getState() != DeployState.SCORE_COMPACTION
              && deploy.getState() != DeployState.SCORE_COMPACTION_WAITING) {
            deploy.intakeRequest();
          }
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
        if (operatorWantsStow) {
          deploy.stowRequest();
          intake.idleRequest();
        } else {
          // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
          if (deploy.getState() != DeployState.FEED_COMPACTION) {
            deploy.intakeRequest();
          }
          intake.feedRequest();
        }
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
  protected void beforeTransition(HopperState oldState, HopperState newState) {
    // A released intake lets the current fill finish. Explicit stow, shooting, eject,
    // and unjam take control immediately and must not leave a fill pending.
    if (newState != HopperState.INTAKING && newState != HopperState.IDLE_DEPLOYED) {
      ballFillActive = false;
      ballFillTimer.stop();
    }
    boolean wasShooting =
        switch (oldState) {
          case SCORE, SCORE_AND_INTAKE, FEED, FEED_AND_INTAKE -> true;
          default -> false;
        };
    boolean willShoot =
        switch (newState) {
          case SCORE, SCORE_AND_INTAKE, FEED, FEED_AND_INTAKE -> true;
          default -> false;
        };
    // With no fuel sensor, retire the estimate after a shooting burst or an eject request.
    if ((wasShooting && !willShoot)
        || newState == HopperState.EJECTING
        || newState == HopperState.UNJAMMING) {
      hopperCapacity = HopperCapacity.LOW;
      // Intake releases do not empty the hopper or rearm filling.
      ballFillAvailable = true;
    }
  }

  @Override
  protected void collectInputs() {
    // Intake stopping does not empty the hopper. Retain the estimate through idle and
    // the next shooting burst so the shooter receives its full-hopper starting boost.
    if (intake.hasBeenIntaking()) {
      hopperCapacity = HopperCapacity.HIGH;
    }
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
    switch (state) {
      case IDLE_DEPLOYED -> {
        updateBallFill();
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
        deploy.intakeRequest();
        intake.intakeRequest();
        updateBallFill();
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
    DogLog.log("HopperManager/IsShooting", isShooting());
  }
}
