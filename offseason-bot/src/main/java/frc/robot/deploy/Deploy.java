package frc.robot.deploy;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private final MotionMagicTorqueCurrentFOC motionMagicRequest =
      new MotionMagicTorqueCurrentFOC(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final NeutralOut neutralRequest = new NeutralOut();
  private double averagePosition = 0.0;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;

  private boolean simulationInitialized = false;

  private double leftStatorCurrent = 0.0;
  private double rightStatorCurrent = 0.0;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> rightStatorCurrentSignal;

  private final StatusSignal<Current> leftSupplyCurrentSignal;

  private final StatusSignal<Current> rightSupplyCurrentSignal;

  public Deploy(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftMotor.getConfigurator().apply(DeployConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(DeployConfig.RIGHT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);

    setControl(neutralRequest);

    leftPositionSignal = leftMotor.getPosition(false);
    rightPositionSignal = rightMotor.getPosition(false);
    leftStatorCurrentSignal = leftMotor.getStatorCurrent(false);
    rightStatorCurrentSignal = rightMotor.getStatorCurrent(false);
    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
    Signals.forDevice(leftMotor)
        .addSignals(leftPositionSignal, leftStatorCurrentSignal, leftSupplyCurrentSignal);
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
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
      default -> setStateFromRequest(DeployState.BEAST_MODE_COMPACTION);
    }
  }

  public void feedCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
      default -> setStateFromRequest(DeployState.FEED_COMPACTION);
    }
  }

  /** Arm angle in degrees: zero is fully down, positive is raised. */
  public double getAngleDegrees() {
    return averagePosition;
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
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION);
    }
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
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
                    .withMaxPosition(-DeployConfig.MIN_ANGLE / 360.0));

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

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {
        // Homing takes priority over hopper requests.
      }
      default -> setStateFromRequest(DeployState.STOW);
    }
  }

  public void waitHopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION_WAITING);
    }
  }

  private void applyOutputs(DeployState newState) {
    // Keep the homing voltage request configured while disabled; the DS inhibits motor output.
    if (DriverStation.isDisabled()
        && newState != DeployState.HOME_INWARD
        && newState != DeployState.HOME_OUTWARD) {
      setControl(neutralRequest);
      return;
    }
    // Deadband: stop driving the intake once it's within tolerance of the intake goal.
    if (newState == DeployState.INTAKE
        && MathUtil.isNear(
            clamp(newState.getAngleDegrees()),
            averagePosition,
            DeployConfig.INTAKE_TOLERANCE_DEGREES)) {
      setControl(voltageRequest.withOutput(0));
      return;
    }
    switch (newState) {
      case UNHOMED, AUTO_INTAKE_DELAY -> setControl(neutralRequest);
      case HOME_INWARD -> setControl(voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_INWARD));
      case HOME_OUTWARD ->
          setControl(voltageRequest.withOutput(DeployConfig.HOMING_VOLTAGE_OUTWARD));
      default ->
          setControl(motionMagicRequest.withPosition(-clamp(newState.getAngleDegrees()) / 360.0));
    }
  }

  private void setControl(ControlRequest request) {
    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  private void setPosition(double angleDegrees) {
    leftMotor.setPosition(-angleDegrees / 360.0);
    rightMotor.setPosition(-angleDegrees / 360.0);
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
    averagePosition = MathHelpers.average(leftMotorPosition, rightMotorPosition);
    leftStatorCurrent = leftStatorCurrentSignal.getValueAsDouble();
    rightStatorCurrent = rightStatorCurrentSignal.getValueAsDouble();
    if (RobotBase.isSimulation() && getState().isHoming()) {
      leftStatorCurrent = simulatedHomingCurrent(leftMotorPosition);
      rightStatorCurrent = simulatedHomingCurrent(rightMotorPosition);
    }
    DogLog.log("Deploy/LeftMotor/StatorCurrent", leftStatorCurrent);
    DogLog.log("Deploy/RightMotor/StatorCurrent", rightStatorCurrent);
    DogLog.log("Deploy/LeftMotor/SupplyCurrent", leftSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Deploy/RightMotor/SupplyCurrent", rightSupplyCurrentSignal.getValueAsDouble());
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
          setPosition(DeployConfig.HOMING_END_POSITION_INWARD);
          yield stateAfterHoming();
        } else {
          yield currentState;
        }
      }

      case HOME_OUTWARD -> {
        if (leftStatorCurrent > DeployConfig.HOMING_CURRENT_OUTWARD
            && rightStatorCurrent > DeployConfig.HOMING_CURRENT_OUTWARD) {
          setPosition(DeployConfig.HOMING_END_POSITION_OUTWARD);
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
    DogLog.log("Deploy/AngleDegrees", averagePosition);
  }
}
