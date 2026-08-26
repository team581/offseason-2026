package frc.robot.kicker;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Kicker extends StateMachineSubsystem<KickerState> implements PowerManaged {
  private final TalonFX motor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public Kicker(TalonFX motor) {
    super(SubsystemPriority.FUNNELER, KickerState.IDLE);
    motor.getConfigurator().apply(KickerConfig.MOTOR_CONFIG);
    this.motor = motor;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(KickerConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }

  public void ballFillingRequest() {
    setStateFromRequest(KickerState.BALL_FILLING);
  }

  public void idleRequest() {
    setStateFromRequest(KickerState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(KickerState.INTAKE);
  }

  public void shootRequest() {
    setStateFromRequest(KickerState.SHOOT);
  }

  @Override
  protected void afterTransition(KickerState newState) {
    switch (newState) {
      case IDLE -> motor.setControl(neutralRequest);
      default -> motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
    }
  }
}
