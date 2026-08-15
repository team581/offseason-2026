package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX intakeMotorR;
  private final TalonFX intakeMotorL;

  public Intake(TalonFX left, TalonFX right) {
    // Create a state machine subsystem, starting in IDLE and running at INTAKE priority
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
     
    this.intakeMotorR = right;
    this.intakeMotorL = left;
  }

  // Request methods are used to ask this subsystem to do a thing
  // Usually this means triggering a manual state transition
  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }


  public void collectRequest() {
    setStateFromRequest(IntakeState.COLLECTING);
  }
  
  
  @Override
  protected void afterTransition(IntakeState newState) {
    // afterTransition runs after a state transition (ex. A -> B)
    // It tells you what the new state is, so that you can trigger logic like setting motor voltages here
    if (newState.equals(IntakeState.IDLE)){
      intakeMotorL.setVoltage(0);
      intakeMotorR.setVoltage(0);
    }else if (newState.equals(IntakeState.COLLECTING)){
      intakeMotorL.setVoltage(10);
      intakeMotorR.setVoltage(10);

    }else{
      intakeMotorL.setVoltage(-10);
      intakeMotorR.setVoltage(-10);
    }
  }
}
