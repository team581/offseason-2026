package frc.robot.robot_manager.hopper_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;

  public final Feeder feeder;
  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;

  private boolean wantsSafeStow = false;

  private boolean ballFilling = false;
  private boolean shouldBeastMode = false;

  public HopperManager(Deploy deploy, Intake intake, Conveyor conveyor, Feeder feeder) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
  }

  public void feedRequest() {
    setState(resolveFeedState());
  }

  public double getFeederToShooterTime() {
    return 0.2;
  }

  public boolean hasBallsInTower() {
    // Without a tower sensor, assume fuel is present for shooting and disable automatic ball
    // filling.
    return true;
  }

  public void idleRequest() {
    setState(resolveIdleState());
  }

  public boolean isFull() {
    return false;
  }

  public boolean isIntaking() {
    return intake.getState() == IntakeState.INTAKE;
  }

  public boolean isShooting() {
    return true;
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
    wantsSafeStow = false;
    driverWantsIntake = wantsIntake;
  }

  public void setOperatorWantsStow(boolean wantsStow) {
    operatorWantsStow = wantsStow;
  }

  public void setWantsSafeStow(boolean wantsSafeStow) {
    this.wantsSafeStow = wantsSafeStow;
  }

  public void unjamRequest() {
    setState(HopperState.UNJAMMING);
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

    if (wantsSafeStow) {
      return HopperState.IDLE_SAFE_KICKER_STOW;
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

  /** Keeps conveyor and feeder stopped because there is no tower sensor to stop ball filling. */
  private void smartBallFillRequest() {
    conveyor.idleRequest();
    feeder.idleRequest();
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
      }
      case IDLE_SAFE_KICKER_STOW -> {
        deploy.safeKickerStowRequest();
        intake.idleRequest();
        smartBallFillRequest();
      }
      case INTAKING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        smartBallFillRequest();
      }
      case EJECTING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.ejectRequest();
        feeder.idleRequest();
      }
      case UNJAMMING -> {
        deploy.intakeRequest();
        intake.ejectRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case SCORE -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (!operatorWantsStow
            && deploy.getState() != DeployState.SCORE_COMPACTION
            && deploy.getState() != DeployState.SCORE_COMPACTION_WAITING) {
          deploy.intakeRequest();
        }
        intake.shootRequest();
        conveyor.initialShotRequest();
        feeder.shootRequest();
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }

      case FEED -> {
        // Don't move deploy back to intake if it's already compacting from a previous SHOOT cycle
        if (deploy.getState() != DeployState.FEED_COMPACTION) {
          deploy.intakeRequest();
        }
        intake.shootRequest();
        conveyor.initialShotRequest();
        feeder.shootRequest();
      }
      case FEED_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
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
    if (state.canBallFill) {
      smartBallFillRequest();
    }

    switch (state) {
      default -> {}
      case SCORE -> {
        if (operatorWantsStow) {
          deploy.stowRequest();
        } else if (shouldBeastMode) {
          deploy.beastModeRequest();
          intake.shootRequest();
          conveyor.shootRequest();
        } else if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          double shuffleInterval =
              HopperManagerConfig.HOPPER_COMPACTION_SHUFFLE_INTERVAL.getAsDouble();
          if (((int) (Timer.getFPGATimestamp() / shuffleInterval)) % 2 == 0) {
            deploy.hopperCompactionRequest();
          } else {
            deploy.intakeRequest();
          }
          intake.idleRequest();
          conveyor.shootRequest();
        } else {
          deploy.waitHopperCompactionRequest();
        }
      }
      case IDLE_STOWED -> {
        deploy.stowRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case FEED -> {
        if (timeout(HopperManagerConfig.HOPPER_COMPACTION_DELAY.getAsDouble())) {
          deploy.feedCompactionRequest();
          intake.idleRequest();
          conveyor.shootRequest();
        }
      }
    }

    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
  }
}
