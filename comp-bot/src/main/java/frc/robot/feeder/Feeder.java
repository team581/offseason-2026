package frc.robot.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> {

  private final TalonFX motor;

  public Feeder(TalonFX motor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    motor.getConfigurator().apply(FeederConfig.MOTOR_CONFIG);

    this.motor = motor;
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOT);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  @Override
  protected void afterTransition(FeederState newState) {
    switch (newState) {
      case IDLE -> {
        motor.disable();
      }
      case SHOOT -> {
        motor.setVoltage(newState.getVoltage());
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Feeder/Left/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Feeder/Left/VelocityRPM", motor.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Left/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Feeder/Voltage", getState().getVoltage());
  }
}
