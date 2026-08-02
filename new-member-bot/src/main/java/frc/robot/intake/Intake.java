package frc.robot.intake;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  public Intake() {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }
}
