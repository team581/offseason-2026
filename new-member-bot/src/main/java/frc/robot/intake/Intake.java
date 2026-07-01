package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private double leftCurrent;
  private double rightCurrent;

  private VoltageOut voltageRequest = new VoltageOut(0.0);

  private TalonFXConfiguration leftConfiguration =
      new TalonFXConfiguration()
          .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20));
  private TalonFXConfiguration rightConfiguration =
      new TalonFXConfiguration()
          .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(20));

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKING, IntakeState.IDLE);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);

    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
  }

  @Override
  public void afterTransition(IntakeState state) {
    if (state == IntakeState.IDLE) {
      setIntakeVoltage(0.0);
    }
    if (state == IntakeState.EJECT) {
      setIntakeVoltage(-12.0);
    }
    if (state == IntakeState.INTAKING) {
      setIntakeVoltage(12.0);
    }
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            IntakeConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));

    rightMotor
        .getConfigurator()
        .apply(
            IntakeConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
  }

  public void haltIntake() {
    DutyCycleOut neutralRequest = null;
    leftMotor.setControl(neutralRequest);
    rightMotor.setControl(neutralRequest);
  }

  // halt or stop I can change name if needed

  public void haltIntakeRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void setIntakeVoltage(double voltage) {
    leftMotor.setControl(voltageRequest.withOutput(voltage));
    rightMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  protected void collectInputs() {
    leftCurrent = leftSupplyCurrentSignal.getValueAsDouble();
    rightCurrent = rightSupplyCurrentSignal.getValueAsDouble();

    DogLog.log("Intake/Left/SupplyCurrent", leftCurrent);
    DogLog.log("Intake/Right/SupplyCurrent", rightCurrent);
  }
}
