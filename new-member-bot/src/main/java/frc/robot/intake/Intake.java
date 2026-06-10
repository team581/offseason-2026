package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;

public class Intake extends StateMachineSubsystem<IntakeState> { // No state machine
  private final TalonFX leftmotor;
  private final TalonFX rightmotor;

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKING, IntakeState.IDLE); // no subsystem priority
    this.leftmotor = leftMotor;
    this.rightmotor = rightMotor;
    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
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
