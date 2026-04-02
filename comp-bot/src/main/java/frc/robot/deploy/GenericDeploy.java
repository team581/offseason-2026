package frc.robot.deploy;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public abstract class GenericDeploy extends StateMachineSubsystem<DeployState>
    implements PowerManaged {
  protected GenericDeploy() {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
  }

  public abstract boolean atGoal();

  public abstract double getPosition();

  public abstract void homeInAutoRequest();

  public abstract void homingRequest();

  public abstract void hopperCompactionRequest();

  public abstract void intakeRequest();

  public abstract boolean isFullyExtended();

  public abstract void stowRequest();
}
