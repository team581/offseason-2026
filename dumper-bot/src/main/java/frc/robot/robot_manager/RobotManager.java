package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.util.scheduling.SubsystemPriority;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private final Intake intake;
  private final Shooter shooter;

  public RobotManager(Intake intake, Shooter shooter) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.intake = intake;
    this.shooter = shooter;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case PREPARE_SHOOT_HUB -> shooter.atGoal() ? RobotState.SHOOT_HUB : currentState;
      case PREPARE_FEED_1 -> shooter.atGoal() ? RobotState.FEED_1 : currentState;
      case PREPARE_FEED_2 -> shooter.atGoal() ? RobotState.FEED_2 : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    // TODO: distance
    switch (newState) {
      case WAIT_FEED_1, WAIT_FEED_2, PREPARE_FEED_1, PREPARE_FEED_2, FEED_1, FEED_2 ->
          shooter.feedRequest(0);
      case PREPARE_SHOOT_HUB, WAIT_SHOOT_HUB, SHOOT_HUB -> shooter.scoreRequest(0);
      default -> shooter.idleRequest();
    }
  }

  private void setStateFailSafe(RobotState newState) {
    if (getState().climbingOrRehoming()) {
      return;
    }
    setStateFromRequest(newState);
  }

  public void idleRequest() {
    setStateFailSafe(RobotState.IDLE);
  }

  public void intakeRequest() {
    intake.intakeRequest();
  }

  public void cancelIntakeRequest() {
    intake.idleRequest();
  }

  public void shootHubWaitRequest() {
    setStateFailSafe(RobotState.WAIT_SHOOT_HUB);
  }

  public void feed1WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_1);
  }

  public void feed2WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_2);
  }

  public void confirmShotRequest() {
    switch (getState()) {
      default -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
      case WAIT_SHOOT_HUB -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
      case WAIT_FEED_1 -> setStateFailSafe(RobotState.PREPARE_FEED_1);
      case WAIT_FEED_2 -> setStateFailSafe(RobotState.PREPARE_FEED_2);
    }
  }

  public void climbSequenceForward() {
    cancelIntakeRequest();
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP);
      case CLIMB_1_LINEUP -> setStateFromRequest(RobotState.CLIMB_2_RAISING);
      case CLIMB_2_RAISING -> setStateFromRequest(RobotState.CLIMB_3_HANGING);
      case CLIMB_3_HANGING -> {}
    }
  }

  public void climbSequenceBackward() {
    cancelIntakeRequest();
    switch (getState()) {
      default -> {}
      case CLIMB_1_LINEUP -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING -> setStateFromRequest(RobotState.CLIMB_1_LINEUP);
      case CLIMB_3_HANGING -> setStateFromRequest(RobotState.CLIMB_2_RAISING);
    }
  }
}
