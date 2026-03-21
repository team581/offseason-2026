package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.hardware.CANrange;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployConfig;
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

  private final LinearFilter hopperFilter = LinearFilter.movingAverage(5);

  private double hopperDistance = 0.0;
  private double filteredDistance = 0.0;
  private double previousCanRangeDistance = 0.0;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;
  private boolean capacityNotHigh = true;

  private final Timer shuffleTimer = new Timer();
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

    shuffleTimer.start();
  }

  @Override
  protected HopperState getNextState(HopperState currentState) {
    if (deploy.getState().atGoal()) {
      return HopperState.SCORE;
    }

    return switch (currentState) {
      case INTAKE ->
          hopperCapacity == HopperCapacity.MEDIUM ? HopperState.BALL_FILLING : currentState;
      case BALL_FILLING -> {
        if (towerSensor.get()) {
          yield HopperState.INTAKE;
        }
        yield currentState;
      }
      case COMPACT_IN -> {
        if (deploy.atGoal()) {
          yield HopperState.IDLE;
        }
        yield currentState;
      }
      default -> currentState;
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
      case SCORE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.intakeRequest();
        feeder.idleRequest();
      }
      case BALL_FILLING -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.intakeRequest();
        feeder.ballFillingRequest();
      }
      case SCORE_AND_INTAKE -> {
        deploy.intakeRequest();
        intake.intakeRequest();
        conveyor.shootRequest();
        feeder.shootRequest();
      }
      case REHOME_DEPLOY -> {
        deploy.homingRequest();
        intake.idleRequest();
        conveyor.idleRequest();
        feeder.idleRequest();
      }

      case COMPACT_IN -> {
        deploy.hopperCompactionRequest();
        conveyor.shootRequest();
        feeder.idleRequest();
      }
    }
  }

  private void setState(HopperState newState) {
    switch (deploy.getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public void intakeRequest() {
    setStateFromRequest(HopperState.INTAKE);
  }

  public void scoreRequest() {
    setStateFromRequest(HopperState.SCORE);
  }

  public void scoreAndIntakeRequest() {
    setStateFromRequest(HopperState.SCORE_AND_INTAKE);
  }

  public void climbRequest() {
    setStateFromRequest(HopperState.CLIMB_EMPTY);
  }

  public void idleRequest() {
    setStateFromRequest(HopperState.IDLE);
  }

  public void rehomeDeployRequest() {
    setStateFromRequest(HopperState.REHOME_DEPLOY);
  }

  @Override
  protected void collectInputs() {
    if (DSOptions.USE_CANRANGE.get()) {
      hopperDistance = Units.metersToInches(hopperCANRange.getDistance().getValueAsDouble());
    }

    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance >= DeployConfig.HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else if (filteredDistance >= DeployConfig.MEDIUM_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.MEDIUM;
    } else {
      hopperCapacity = HopperCapacity.LOW;
    }

    capacityNotHigh =
        !DSOptions.USE_CANRANGE.getAsBoolean() || hopperCapacity != HopperCapacity.HIGH;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    if (previousCanRangeDistance != hopperDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperDistance;

    if (canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
  }
}
