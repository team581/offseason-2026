package frc.robot.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> implements PowerManaged {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
    Signals.forDevice(leftMotor).addSignals(leftSupplyCurrentSignal);
    Signals.forDevice(rightMotor).addSignals(rightSupplyCurrentSignal);
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
  }

  public void shootRequest() {
    setStateFromRequest(IntakeState.SHOOT);
  }

  public boolean hasBeenIntaking() {
    if (getState() == IntakeState.INTAKE && timeout(3)) {
      return true;
    }
    return false;
  }

  public void stopShootingRequest() {
    switch (getState()) {
      case SHOOT -> setStateFromRequest(IntakeState.IDLE);
      default -> {}
    }
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
      }
      default -> {
        leftMotor.setControl(voltageRequest.withOutput(newState.voltage));
        rightMotor.setControl(voltageRequest.withOutput(newState.voltage));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Intake/Left/SupplyCurrent", leftSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Intake/Right/SupplyCurrent", rightSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Intake/HasBeenIntaking", hasBeenIntaking());
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
}
