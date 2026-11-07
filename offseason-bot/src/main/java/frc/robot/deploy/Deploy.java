package frc.robot.deploy;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> implements PowerManaged {
  private static double clamp(double angleDegrees) {
    return MathUtil.clamp(angleDegrees, DeployConfig.MIN_ANGLE, DeployConfig.MAX_ANGLE);
  }

  private static double goalTolerance(DeployState state) {
    return state == DeployState.INTAKE
        ? DeployConfig.INTAKE_TOLERANCE_DEGREES
        : DeployConfig.POSITION_TOLERANCE;
  }

  private final TalonFX leftMotor;

  private final TalonFX rightMotor;

  private final DifferentialMechanism<TalonFX> differentialMechanism;

  private final PositionTorqueCurrentFOC torqueCurrentFOCAverageRequest =
      new PositionTorqueCurrentFOC(0).withSlot(0);

  private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOCAverageRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final PositionTorqueCurrentFOC positionDifferentialRequest =
      new PositionTorqueCurrentFOC(0).withSlot(1);

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final VoltageOut voltageDifferentialRequest = new VoltageOut(0).withEnableFOC(false);
  private double differentialMechanismPosition = 0.0;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;

  private boolean simulationInitialized = false;

  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<Angle> differentialAveragePositionSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> rightStatorCurrentSignal;

  private final StatusSignal<Current> leftSupplyCurrentSignal;

  private final StatusSignal<Current> rightSupplyCurrentSignal;

  public Deploy(DifferentialMechanism<TalonFX> differentialMechanism) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.differentialMechanism = differentialMechanism;
    this.leftMotor = differentialMechanism.getLeader();
    this.rightMotor = differentialMechanism.getFollower();
    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Left/Slot1", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG.Slot1);
    TunablePid.register("Deploy/Left/Slot2", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG.Slot2);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right/Slot1", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG.Slot1);
    TunablePid.register("Deploy/Right/Slot2", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG.Slot2);

    differentialMechanism.setNeutralOut();

    leftPositionSignal = leftMotor.getPosition(false);
    rightPositionSignal = rightMotor.getPosition(false);
    differentialAveragePositionSignal = differentialMechanism.getAveragePosition(false);
    leftStatorCurrentSignal = leftMotor.getStatorCurrent(false);
    rightStatorCurrentSignal = rightMotor.getStatorCurrent(false);
    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
    Signals.forDevice(leftMotor)
        .addSignals(
            leftPositionSignal,
            differentialAveragePositionSignal,
            leftStatorCurrentSignal,
            leftSupplyCurrentSignal);
    Signals.forDevice(rightMotor)
        .addSignals(rightPositionSignal, rightStatorCurrentSignal, rightSupplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            DeployConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            DeployConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public boolean atGoal() {
    return atGoal(getState());
  }

  public boolean atGoal(DeployState state) {
    return switch (state) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> false;
      default ->
          MathUtil.isNear(clamp(state.getAngleDegrees()), leftMotorPosition, goalTolerance(state))
              && MathUtil.isNear(
                  clamp(state.getAngleDegrees()), rightMotorPosition, goalTolerance(state));
    };
  }

  public void beastModeRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {}
      default -> setStateFromRequest(DeployState.BEAST_MODE_COMPACTION);
    }
  }

  public void feedCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {}
      default -> setStateFromRequest(DeployState.FEED_COMPACTION);
    }
  }

  public void fixDifferentialDesyncRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
      default -> setStateFromRequest(DeployState.FIX_DIFFERENTIAL_DESYNC);
    }
  }

  /** Arm angle in degrees: zero is fully down, positive is raised. */
  public double getAngleDegrees() {
    return differentialMechanismPosition;
  }

  public void homeInAutoRequest() {
    setStateFromRequest(DeployState.HOME_INWARD);
  }

  public void homingRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(DeployState.HOME_INWARD);
    } else {
      setStateFromRequest(DeployState.HOME_OUTWARD);
    }
  }

  public void hopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION);
    }
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {}
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public boolean isFullyExtended() {
    return atGoal(DeployState.INTAKE);
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "Deploy",
            mechanism ->
                mechanism
                    .addMotor(leftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(rightMotor, ChassisReference.CounterClockwise_Positive)
                    .withMinPosition(-DeployConfig.HOMING_END_POSITION_INWARD / 360.0)
                    .withMaxPosition(-DeployConfig.HOMING_END_POSITION_OUTWARD / 360.0));

    if (!simulationInitialized) {
      // The robot is placed on the field with its intake raised. This seeds only the physical
      // simulation; homing still requires enabled motor current at the stop.
      deploySimulation.seedPosition(-DeployConfig.HOMING_END_POSITION_INWARD / 360.0);
      simulationInitialized = true;
      return;
    }

    double target =
        switch (getState()) {
          case HOME_INWARD -> DeployConfig.HOMING_END_POSITION_INWARD;
          case HOME_OUTWARD -> DeployConfig.HOMING_END_POSITION_OUTWARD;
          default -> clamp(getState().getAngleDegrees());
        };
    if (getState() == DeployState.UNHOMED || getState() == DeployState.AUTO_INTAKE_DELAY) {
      target = getAngleDegrees();
    }
    deploySimulation.update(-target / 360.0);
  }

  /** Release the operator override so normal hopper requests can resume. */
  public void stopFixDifferentialDesyncRequest() {
    if (getState() == DeployState.FIX_DIFFERENTIAL_DESYNC) {
      setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {
        // Homing and the operator desync override take priority over hopper requests.
      }
      default -> setStateFromRequest(DeployState.STOW);
    }
  }

  public void waitHopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY, FIX_DIFFERENTIAL_DESYNC -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION_WAITING);
    }
  }

  private void applyOutputs(DeployState newState) {
    // Keep the homing voltage request configured while disabled; the DS inhibits motor output.
    if (DriverStation.isDisabled()
        && newState != DeployState.HOME_INWARD
        && newState != DeployState.HOME_OUTWARD) {
      differentialMechanism.setNeutralOut();
      return;
    }
    switch (newState) {
      case UNHOMED, AUTO_INTAKE_DELAY -> differentialMechanism.setNeutralOut();
      case HOME_INWARD ->
          differentialMechanism.setControl(
              voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_INWARD),
              voltageDifferentialRequest.withOutput(0));
      case HOME_OUTWARD ->
          differentialMechanism.setControl(
              voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_OUTWARD),
              voltageDifferentialRequest.withOutput(0));
      case SCORE_COMPACTION_WAITING -> {
        differentialMechanism.setControl(
            motionMagicTorqueCurrentFOCAverageRequest
                .withPosition(-clamp(newState.getAngleDegrees()) / 360.0)
                .withSlot(0),
            positionDifferentialRequest.withPosition(0.0).withSlot(1));
      }
      case FIX_DIFFERENTIAL_DESYNC -> {
        differentialMechanism.setControl(
            torqueCurrentFOCAverageRequest
                .withPosition(-clamp(newState.getAngleDegrees()) / 360.0)
                .withSlot(0),
            positionDifferentialRequest.withPosition(0.0).withSlot(2));
      }
      default ->
          differentialMechanism.setControl(
              motionMagicTorqueCurrentFOCAverageRequest
                  .withPosition(-clamp(newState.getAngleDegrees()) / 360.0)
                  .withSlot(0),
              positionDifferentialRequest.withPosition(0.0).withSlot(1));
    }
  }

  private double simulatedHomingCurrent(double angle) {
    if (DriverStation.isDisabled() || !simulationInitialized) {
      return 0.0;
    }
    double voltage;
    double stop;
    switch (getState()) {
      case HOME_INWARD -> {
        voltage = DeployConfig.HOMING_VOLTAGE_INWARD;
        stop = DeployConfig.HOMING_END_POSITION_INWARD;
      }
      case HOME_OUTWARD -> {
        voltage = DeployConfig.HOMING_VOLTAGE_OUTWARD;
        stop = DeployConfig.HOMING_END_POSITION_OUTWARD;
      }
      default -> {
        return 0.0;
      }
    }
    // SimKit supplies kinematic positions, but does not model hard-stop motor load.
    // Feed the contact current into the same detector used on the physical robot.
    return MathUtil.isNear(stop, angle, 0.5)
        ? Math.abs(DCMotor.getKrakenX60(1).getCurrent(0.0, voltage))
        : 0.0;
  }

  private DeployState stateAfterHoming() {
    return DriverStation.isAutonomous() ? DeployState.AUTO_INTAKE_DELAY : DeployState.INTAKE;
  }

  @Override
  protected void afterTransition(DeployState newState) {
    applyOutputs(newState);
  }

  @Override
  protected void collectInputs() {
    leftMotorPosition = leftPositionSignal.getValueAsDouble() * -360.0;
    rightMotorPosition = rightPositionSignal.getValueAsDouble() * -360.0;
    if (RobotBase.isSimulation()) {
      // The firmware's DifferentialAveragePosition signal isn't reliably computed in sim,
      // so derive it from individual motor positions instead.
      differentialMechanismPosition = MathHelpers.average(leftMotorPosition, rightMotorPosition);
    } else {
      differentialMechanismPosition = differentialAveragePositionSignal.getValueAsDouble() * -360.0;
    }
    leftStatorCurrent = leftStatorCurrentSignal.getValueAsDouble();
    rightStatorCurrent = rightStatorCurrentSignal.getValueAsDouble();
    if (RobotBase.isSimulation() && getState().isHoming()) {
      leftStatorCurrent = simulatedHomingCurrent(leftMotorPosition);
      rightStatorCurrent = simulatedHomingCurrent(rightMotorPosition);
    }
    DogLog.log("Deploy/LeftMotor/StatorCurrent", leftStatorCurrent);
    DogLog.log("Deploy/RightMotor/StatorCurrent", rightStatorCurrent);
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    if (DriverStation.isDisabled()) {
      if (currentState == DeployState.AUTO_INTAKE_DELAY) {
        resetTimeout();
      }
      // Disabled time must not count toward the auto intake delay.
      return currentState;
    }
    return switch (currentState) {
      case AUTO_INTAKE_DELAY ->
          timeout(DeployConfig.AUTO_INTAKE_DELAY_SECONDS) ? DeployState.INTAKE : currentState;
      case HOME_INWARD -> {
        if (leftStatorCurrent > DeployConfig.HOMING_CURRENT
            && rightStatorCurrent > DeployConfig.HOMING_CURRENT) {
          differentialMechanism.setPosition(-DeployConfig.HOMING_END_POSITION_INWARD / 360.0);
          yield stateAfterHoming();
        } else {
          yield currentState;
        }
      }

      case HOME_OUTWARD -> {
        if (leftStatorCurrent > DeployConfig.HOMING_CURRENT_OUTWARD
            && rightStatorCurrent > DeployConfig.HOMING_CURRENT_OUTWARD) {
          differentialMechanism.setPosition(-DeployConfig.HOMING_END_POSITION_OUTWARD / 360.0);
          yield stateAfterHoming();
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(DeployState state) {
    applyOutputs(state);
    DogLog.log("Deploy/LeftMotor/AngleDegrees", leftMotorPosition);
    DogLog.log("Deploy/RightMotor/AngleDegrees", rightMotorPosition);
    DogLog.log("Deploy/GoalAngleDegrees", getState().getAngleDegrees());
    DogLog.log("Deploy/AngleDegrees", differentialMechanismPosition);
  }
}
