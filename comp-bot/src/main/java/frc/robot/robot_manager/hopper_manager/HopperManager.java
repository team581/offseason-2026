package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.hardware.CANrange;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;
  public final Feeder feeder;
  public final CANrange hopperCANRange;
  public final DigitalInput towerSensor;

  private final Debouncer towerSensorDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;

  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;
  private boolean towerSensorRaw = false;

  private final LinearFilter hopperFilter = LinearFilter.movingAverage(5);

  private double hopperDistance = 0.0;
  private double filteredDistance = 0.0;
  private double previousCanRangeDistance = 0.0;
  public static double HIGH_CAPACITY_THRESHOLD = 10;
  public static double MEDIUM_CAPACITY_THRESHOLD = 5;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  private final Timer canRangeUpdateTimer = new Timer();

  public HopperManager(
      Deploy deploy,
      Intake intake,
      Conveyor conveyor,
      Feeder feeder,
      CANrange hopperCANRange,
      DigitalInput towerSensor) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.hopperCANRange = hopperCANRange;
    this.towerSensor = towerSensor;
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    return switch (currentState) {
      case SHOOT -> currentState;
      case IDLE -> {
        if ((intake.hasBeenIntaking()
                || hopperCapacity == HopperCapacity.MEDIUM
                || hopperCapacity == HopperCapacity.HIGH)
            && !towerSensorRaw) {
          yield HopperState.BALL_FILLING;
        }
        yield currentState;
      }
      case BALL_FILLING -> {
        if (towerSensorRaw) {
          yield HopperState.IDLE;
        }
        yield currentState;
      }
      case REHOME_DEPLOY -> deploy.getState().isHoming() ? currentState : HopperState.IDLE;
    };
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case IDLE -> {
        deploy.intakeRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
      case SHOOT -> {
        deploy.hopperCompactionRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case BALL_FILLING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.ballFillingRequest();
        feeder.ballFillingRequest();
      }
      case REHOME_DEPLOY -> {
        deploy.homingRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }
    }
  }

  @Override
  protected void whileInState(HopperState state) {
    switch (state) {
      default -> {}
      case BALL_FILLING -> {
        if (driverWantsEject) {
          DogLog.log("HopperManager/HopperActivity", "EJECTING");
          deploy.intakeRequest();
          intake.ejectRequest();
          conveyor.ballFillingRequest();
          feeder.idleRequest();
        } else if (driverWantsIntake) {
          DogLog.log("HopperManager/HopperActivity", "BALL FILLING");
          deploy.intakeRequest();
          intake.intakeRequest();
          conveyor.ballFillingRequest();
          feeder.ballFillingRequest();
        } else {
          DogLog.log("HopperManager/HopperActivity", "IDLE_FILLING");
          deploy.intakeRequest();
          intake.idleRequest();
          conveyor.ballFillingRequest();
          feeder.ballFillingRequest();
        }
      }
      case IDLE -> {
        if (driverWantsEject) {
          DogLog.log("HopperManager/HopperActivity", "EJECTING");
          deploy.intakeRequest();
          intake.ejectRequest();
          conveyor.ejectRequest();
          feeder.idleRequest();
        } else if (driverWantsIntake) {
          DogLog.log("HopperManager/HopperActivity", "INTAKING");
          conveyor.intakeRequest();
          intake.intakeRequest();
          deploy.intakeRequest();
          feeder.idleRequest();
          if (operatorWantsStow) {
            DogLog.log("HopperManager/HopperActivity", "INTAKING_AND_STOW");
            conveyor.intakeRequest();
            intake.intakeRequest();
            deploy.stowRequest();
            feeder.idleRequest();
          }
        } else if (operatorWantsStow) {
          DogLog.log("HopperManager/HopperActivity", "STOW");
          conveyor.idleRequest();
          intake.idleRequest();
          deploy.stowRequest();
          feeder.idleRequest();
        } else {
          DogLog.log("HopperManager/HopperActivity", "IDLE");
          conveyor.idleRequest();
          intake.idleRequest();
          deploy.intakeRequest();
          feeder.idleRequest();
        }
      }
      case SHOOT -> {
        if (driverWantsEject) {
          deploy.intakeRequest();
          intake.ejectRequest();
          conveyor.ejectRequest();
          feeder.idleRequest();
        } else {
          feeder.shootRequest();
          conveyor.shootRequest();
          intake.intakeRequest();
          if (driverWantsIntake) {
            deploy.intakeRequest();
          } else {
            deploy.hopperCompactionRequest();
          }
        }
      }
    }
    if (previousCanRangeDistance != hopperDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperDistance;

    if (canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
    DogLog.log("HopperManager/State", getState());
    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.log("HopperManager/FilteredHopperDistance", filteredDistance);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
  }

  private void setState(HopperState newState) {
    switch (deploy.getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public boolean stillShooting() {
    return towerSensorDebounced && getState() == HopperState.SHOOT;
  }

  public void scoreRequest() {
    setState(HopperState.SHOOT);
  }

  public void idleRequest() {
    setState(HopperState.IDLE);
  }

  public void rehomeDeployRequest() {
    setStateFromRequest(HopperState.REHOME_DEPLOY);
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

  public boolean isIntaking() {
    return driverWantsIntake && !driverWantsEject;
  }

  @Override
  protected void collectInputs() {
    towerSensorRaw = towerSensor.get();
    towerSensorDebounced = towerSensorDebouncer.calculate(towerSensorRaw);
    if (DSOptions.USE_CANRANGE.get()) {
      hopperDistance = Units.metersToInches(hopperCANRange.getDistance().getValueAsDouble());
    }

    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance >= HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else if (filteredDistance >= MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }
  }
}
