package frc.robot.deploy;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  public Deploy() {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
  }
}
