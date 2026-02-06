package frc.robot.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.math.MathUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Climber extends StateMachineSubsystem<ClimberState> {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);
  private double motorPosition;

  public Climber(TalonFX motor) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);

    this.motor = motor;

    motor.getConfigurator().apply(ClimberConfig.MOTOR_CONFIG);

    motor.setPosition(0);

    TunablePid.register("Climber", this.motor, ClimberConfig.MOTOR_CONFIG);
  }

  public boolean atGoal() {
    return MathUtil.isNear(getState().height, motorPosition, ClimberConfig.TOLERANCE);
  }

  public void l1HangingRequest() {
    setStateFromRequest(ClimberState.L1_HANGING);
  }

  public void l1LineupRequest() {
    setStateFromRequest(ClimberState.L1_LINEUP);
  }

  public void l2HangingRequest() {
    setStateFromRequest(ClimberState.L2_HANGING);
  }

  public void l2LineupRequest() {
    setStateFromRequest(ClimberState.L2_LINEUP);
  }

  public void l3HangingRequest() {
    setStateFromRequest(ClimberState.L3_HANGING);
  }

  public void l3LineupRequest() {
    setStateFromRequest(ClimberState.L3_LINEUP);
  }

  public void stowRequest() {
    setStateFromRequest(ClimberState.STOWED);
  }

  @Override
  protected void afterTransition(ClimberState newState) {
    motor.setControl(positionRequest.withPosition(newState.height));
  }

  @Override
  protected void collectInputs() {
    motorPosition = motor.getPosition().getValueAsDouble();
  }
}
