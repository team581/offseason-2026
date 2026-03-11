package frc.robot.climber;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public abstract class GenericClimber extends StateMachineSubsystem<ClimberState> {
  protected GenericClimber() {
    super(SubsystemPriority.CLIMBER, ClimberState.STOWED);
  }

  public abstract boolean atGoal();

  public abstract double getHeight();

  public abstract void l1HangingRequest();

  public abstract void l1LineupRequest();

  public abstract void l2HangingRequest();

  public abstract void l2LineupRequest();

  public abstract void l3HangingRequest();

  public abstract void l3LineupRequest();

  public abstract void stowRequest();
}
