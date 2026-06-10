package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;

public class Intake extends StateMachineSubsystem<IntakeState> { // No state machine
  private final TalonFX leftmotor;
  private final TalonFX rightmotor;

  public Intake(TalonFX leftmotor, TalonFX rightmotor) {
    super(SubsystemPriority.INTAKING, IntakeState.IDLE); // no subsystem priority
    this.leftmotor = leftmotor;
    this.rightmotor = rightmotor;
    leftmotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightmotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
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
}
