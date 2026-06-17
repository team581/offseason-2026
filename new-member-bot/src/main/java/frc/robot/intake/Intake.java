package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
      haltIntake();
    } else {
      // setIntakeVoltage();
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
  public void haltshootingRequest() {
    if (getState() == IntakeState.SHOOTING) {
      setStateFromRequest(IntakeState.INTAKING);
    }
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void shootRequest() {
    setStateFromRequest(IntakeState.SHOOTING);
  }

  // public void setIntakeVoltage(double voltage) {
  // leftMotor.setControl();
  // rightMotor.setControl();
  // }

  @Override
  protected void collectInputs() {
    leftCurrent = leftSupplyCurrentSignal.getValueAsDouble();
    rightCurrent = rightSupplyCurrentSignal.getValueAsDouble();

    DogLog.log("Intake/Left/SupplyCurrent", leftCurrent);
    DogLog.log("Intake/Right/SupplyCurrent", rightCurrent);
  }
}
