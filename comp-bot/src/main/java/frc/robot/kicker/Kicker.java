package frc.robot.kicker;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Kicker extends StateMachineSubsystem<KickerState> implements PowerManaged {

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public Kicker(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.KICKER, KickerState.IDLE);
    leftMotor.getConfigurator().apply(KickerConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(KickerConfig.RIGHT_MOTOR_CONFIG);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void shootRequest() {
    setStateFromRequest(KickerState.SHOOT);
  }

  public void idleRequest() {
    setStateFromRequest(KickerState.IDLE);
  }

  @Override
  protected void afterTransition(KickerState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case SHOOT -> {
        leftMotor.setVoltage(newState.getVoltage());
        rightMotor.setVoltage(newState.getVoltage());
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Kicker/Left/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Kicker/Left/VelocityRPM", leftMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Kicker/Right/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Kicker/Left/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Kicker/Right/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Kicker/Right/VelocityRPM", rightMotor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Kicker/Voltage", getState().getVoltage());
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            KickerConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            KickerConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
