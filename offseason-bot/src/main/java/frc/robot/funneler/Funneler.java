package frc.robot.funneler;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Funneler extends StateMachineSubsystem<FunnelerState> implements PowerManaged {
  private final TalonFX motor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public Funneler(TalonFX motor) {
    super(SubsystemPriority.FUNNELER, FunnelerState.IDLE);
    motor.getConfigurator().apply(FunnelerConfig.MOTOR_CONFIG);
    this.motor = motor;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(
            FunnelerConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }

  public void ballFillingRequest() {
    setStateFromRequest(FunnelerState.BALL_FILLING);
  }

  public void feedRequest() {
    setStateFromRequest(FunnelerState.FEED);
  }

  public void idleRequest() {
    setStateFromRequest(FunnelerState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FunnelerState.INTAKE);
  }

  public void scoreRequest() {
    setStateFromRequest(FunnelerState.SCORE);
  }

  @Override
  protected void afterTransition(FunnelerState newState) {
    switch (newState) {
      case IDLE -> motor.setControl(neutralRequest);
      default -> motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
    }
  }
}
