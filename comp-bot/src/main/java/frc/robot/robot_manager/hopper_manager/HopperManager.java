package frc.robot.robot_manager.hopper_manager;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.DSOptions;
import frc.robot.config.RobotKind;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.util.scheduling.SubsystemPriority;

public class HopperManager extends StateMachineSubsystem<HopperState> {
  public enum HopperBallPosition {
    CLOSE_TO_SHOOTER,
    AT_SENSOR,
    BELOW_SENSOR,
  }

  public static final double HIGH_CAPACITY_THRESHOLD = 4.0;
  public final Deploy deploy;
  public final Intake intake;
  public final Conveyor conveyor;

  public final Feeder feeder;
  public final CANrange hopperCANRange;

  public final DigitalInput towerSensor;
  private final Debouncer towerSensorDebouncer = new Debouncer(0.25, DebounceType.kFalling);
  private boolean towerSensorDebounced = false;
  private boolean driverWantsIntake = false;
  private boolean driverWantsEject = false;
  private boolean operatorWantsStow = false;

  private boolean wantsSafeStow = false;

  private boolean towerSensorRaw = false;

  private boolean ballFilling = false;
  private boolean shouldBeastMode = false;
  private final LinearFilter hopperFilter = LinearFilter.movingAverage(50);
  private double hopperDistance = 0.0;
  private double filteredDistance = 0.0;
  private double hopperSignalStrength = 0.0;
  private boolean hopperSignalStrengthHigh = false;

  private double previousCanRangeDistance = 0.0;

  private HopperCapacity hopperCapacity = HopperCapacity.LOW;

  private final Timer canRangeUpdateTimer = new Timer();

  private final StatusSignal<Distance> hopperDistanceSignal;
  private final StatusSignal<Double> hopperSignalStrengthSignal;

  public HopperManager(
      Deploy deploy,
      Intake intake,
      Conveyor conveyor,
      Feeder feeder,
      CANrange hopperCANRange,
      DigitalInput towerSensor) {
    super(SubsystemPriority.HOPPER_MANAGER, HopperState.IDLE_DEPLOYED);
    this.deploy = deploy;
    this.intake = intake;
    this.conveyor = conveyor;
    this.feeder = feeder;
    this.hopperCANRange = hopperCANRange;
    this.towerSensor = towerSensor;

    hopperCANRange.getConfigurator().apply(HopperManagerConfig.CAN_RANGE_CONFIG);
    canRangeUpdateTimer.start();

    hopperDistanceSignal = hopperCANRange.getDistance(false);
    hopperSignalStrengthSignal = hopperCANRange.getSignalStrength(false);
    Signals.forDevice(hopperCANRange).addSignals(hopperDistanceSignal, hopperSignalStrengthSignal);
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

  public void idleRequest() {
    setState(resolveIdleState());
  }

  public boolean isFull() {
    if (RobotBase.isSimulation()) {
      return timeout(10.0);
    }
    return hopperCapacity == HopperCapacity.HIGH && DSOptions.USE_CANRANGE.getAsBoolean();
  }

  public boolean isIntaking() {
    return intake.getState() == IntakeState.INTAKE;
  }

  public boolean isShooting() {
    // You need to actually be in a shooting state
    if (getState() != HopperState.SCORE
        && getState() != HopperState.SCORE_AND_INTAKE
        && getState() != HopperState.FEED
        && getState() != HopperState.FEED_AND_INTAKE) {
      return true;
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

  private boolean shouldFillBalls() {
    if (towerSensorDebounced) {
      DogLog.timestamp("HopperManager/Fault/TowerSensor");

      // The sensor in the tower shows we are holding fuel, so we can't fill anymore
      return false;
    }
    if (!deploy.isFullyExtended()) {
      DogLog.timestamp("HopperManager/Fault/DeployNotExtended");
      return false;
    }
    if (!canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      if (!hopperSignalStrengthHigh) {
        DogLog.timestamp("HopperManager/Fault/LowSignalStrength");
        return false;
      }
      DogLog.timestamp("HopperManager/Fault/CheckingCapacity");

      // If we are using the hopper CANrange, we can start filling the tower once the hopper is
      // starting ot fill up
      return hopperCapacity == HopperCapacity.HIGH;
    }
    DogLog.timestamp("HopperManager/Fault/NotUsingRange");

    // Otherwise, we fallback to running once we've been intaking for a few seconds
    return intake.hasBeenIntaking();
  }

  /** Sets conveyor and feeder to ball filling if conditions are met, otherwise idles them. */
  private void smartBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
    } else {
      conveyor.idleRequest();
      feeder.idleRequest();
    }
  }

  private void smartIntakeBallFillRequest() {
    if (shouldFillBalls()) {
      conveyor.ballFillingRequest();
      feeder.ballFillingRequest();
    } else {
      if (towerSensorDebounced) {
        conveyor.idleRequest();
        feeder.idleRequest();
      } else {
        conveyor.intakeRequest();
        feeder.intakeRequest();
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
      }
      case IDLE_SAFE_KICKER_STOW -> {
        deploy.safeKickerStowRequest();
        intake.idleRequest();
        smartBallFillRequest();
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
  protected void collectInputs() {
    if (RobotBase.isSimulation()) {
      hopperDistance = 20;
      hopperSignalStrength = 2000.0;
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

    if (DSOptions.USE_CANRANGE.get()) {
      hopperDistance = Units.metersToInches(hopperDistanceSignal.getValueAsDouble());
      hopperSignalStrength = hopperSignalStrengthSignal.getValueAsDouble();
      hopperSignalStrengthHigh =
          hopperSignalStrength > HopperManagerConfig.MIN_SIGNAL_STRENGTH_VALUE.getAsDouble();
    }
    filteredDistance = hopperFilter.calculate(hopperDistance);

    if (filteredDistance <= HIGH_CAPACITY_THRESHOLD) {
      hopperCapacity = HopperCapacity.HIGH;
    } else {
      hopperCapacity = HopperCapacity.LOW;
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
      if (state == HopperState.INTAKING) {
        smartIntakeBallFillRequest();
      } else {

        smartBallFillRequest();
      }
    }

    switch (state) {
      default -> {}
      case SCORE -> {
        if (operatorWantsStow) {
          deploy.stowRequest();
        } else if (shouldBeastMode) {
          double shuffleInterval = HopperManagerConfig.BEAST_MODE_SHUFFLE_INTERVAL.getAsDouble();
          if (((int) (Timer.getFPGATimestamp() / shuffleInterval)) % 2 == 0) {
            deploy.beastModeCompactionRequest();
          } else {
            deploy.beastModeIntakeRequest();
          }
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

    if (previousCanRangeDistance != hopperDistance) {
      canRangeUpdateTimer.reset();
    }
    previousCanRangeDistance = hopperDistance;

    if (canRangeUpdateTimer.hasElapsed(3.0) && DSOptions.USE_CANRANGE.get()) {
      DogLog.logFault("CANrange distance not updating", AlertType.kError);
    } else {
      DogLog.clearFault("CANrange distance not updating");
    }
    DogLog.log("HopperManager/DriverWantsEject", driverWantsEject);
    DogLog.log("HopperManager/DriverWantsIntake", driverWantsIntake);
    DogLog.log("HopperManager/OperatorWantsStow", operatorWantsStow);
    DogLog.forceNt.log("HopperManager/FilteredHopperDistance", filteredDistance);
    DogLog.log("HopperManager/HopperCapacity", hopperCapacity);
    DogLog.forceNt.log("HopperManager/TowerSensor", towerSensorRaw);
    DogLog.forceNt.log("HopperManager/SignalStrength", hopperSignalStrength);
    DogLog.forceNt.log("HopperManager/SignalStrengthHigh", hopperSignalStrengthHigh);
  }
}
