package frc.robot.deploy;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> implements PowerManaged {
  private static double clamp(double deployLength) {
    return MathUtil.clamp(deployLength, DeployConfig.MIN_LENGTH, DeployConfig.MAX_LENGTH);
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
  private double leftStatorCurrent = 0.0;

  private double rightStatorCurrent = 0.0;
  private final StatusSignal<Angle> leftPositionSignal;
  private final StatusSignal<Angle> rightPositionSignal;
  private final StatusSignal<Angle> differentialAveragePositionSignal;
  private final StatusSignal<Current> leftStatorCurrentSignal;
  private final StatusSignal<Current> rightStatorCurrentSignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;

  private final StatusSignal<Current> rightSupplyCurrentSignal;

  public Deploy(
      DifferentialMechanism<TalonFX> differentialMechanism, TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.differentialMechanism = differentialMechanism;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
    differentialMechanism.setControl(
        torqueCurrentFOCAverageRequest.withPosition(0),
        positionDifferentialRequest.withPosition(0.0));
    differentialMechanism.setControl(positionDifferentialRequest.withPosition(0.0));
    differentialMechanism.setNeutralOut();

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
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> false;
      default ->
          MathUtil.isNear(state.getLength(), leftMotorPosition, DeployConfig.POSITION_TOLERANCE)
              && MathUtil.isNear(
                  state.getLength(), rightMotorPosition, DeployConfig.POSITION_TOLERANCE);
    };
  }

  public void feedCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.FEED_COMPACTION);
    }
  }

  public void fixAlignmentRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.FIX_ALIGNMENT);
    }
  }

  public double getPosition() {
    return differentialMechanismPosition;
  }

  // public void homeInAutoRequest() {
  //  setStateFromRequest(DeployState.HOME_INWARD);
  // }

  public void homingRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(DeployState.HOME_INWARD);
    }
    setStateFromRequest(DeployState.HOME_OUTWARD);
  }

  public void hopperCompactionRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {}
      default -> setStateFromRequest(DeployState.SCORE_COMPACTION);
    }
  }

  public void intakeRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public boolean isFullyExtended() {
    return atGoal(DeployState.INTAKE);
  }

  public void stowRequest() {
    switch (getState()) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> { // Not doing Anything B/c Not Homed
      }
      default -> setStateFromRequest(DeployState.STOW);
    }
  }
}
