package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class RobotManager extends StateMachineSubsystem<RobotState> {
    public RobotManager() {
        super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    }

    @Override
    protected RobotState getNextState(RobotState currentState) {
        return switch (currentState) {
            case PREPARE_SHOOT_HUB -> RobotState.SHOOT_HUB;
            case PREPARE_FEED_1 -> RobotState.FEED_1;
            case PREPARE_FEED_2 -> RobotState.FEED_2;

            case PREPARE_INTAKE_AND_SHOOT_HUB -> RobotState.INTAKE_AND_SHOOT_HUB;
            case PREPARE_INTAKE_AND_FEED_1 -> RobotState.INTAKE_AND_FEED_1;
            case PREPARE_INTAKE_AND_FEED_2 -> RobotState.INTAKE_AND_FEED_2;
            default -> currentState;
        };
    }

    @Override
    protected void afterTransition(RobotState newState) {
        switch (newState) {
            default -> {}
        }
    }

    private void setStateFailSafe(RobotState newState) {
        if (getState().climbingOrRehoming) {
            return;
        }
        setStateFromRequest(newState);
    }

    public void idleRequest() {
        setStateFailSafe(RobotState.IDLE);
    }

    public void intakeRequest() {
        switch (getState()) {
            case SHOOT_HUB -> setStateFailSafe(RobotState.INTAKE_AND_SHOOT_HUB);
            case FEED_1 -> setStateFailSafe(RobotState.INTAKE_AND_FEED_1);
            case FEED_2 -> setStateFailSafe(RobotState.INTAKE_AND_FEED_2);

            case PREPARE_SHOOT_HUB -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_SHOOT_HUB);
            case PREPARE_FEED_1 -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_FEED_1);
            case PREPARE_FEED_2 -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_FEED_2);

            case WAIT_SHOOT_HUB -> setStateFailSafe(RobotState.WAIT_INTAKE_AND_SHOOT_HUB);
            case WAIT_FEED_1 -> setStateFailSafe(RobotState.WAIT_INTAKE_AND_FEED_1);
            case WAIT_FEED_2 -> setStateFailSafe(RobotState.WAIT_INTAKE_AND_FEED_2);
            default -> setStateFailSafe(RobotState.INTAKE);
        }
    }

    public void cancelIntakeRequest() {
        switch (getState()) {
            case WAIT_INTAKE_AND_SHOOT_HUB -> setStateFailSafe(RobotState.WAIT_SHOOT_HUB);
            case WAIT_INTAKE_AND_FEED_1 -> setStateFailSafe(RobotState.WAIT_FEED_1);
            case WAIT_INTAKE_AND_FEED_2 -> setStateFailSafe(RobotState.WAIT_FEED_2);

            case INTAKE_AND_SHOOT_HUB -> setStateFailSafe(RobotState.SHOOT_HUB);
            case INTAKE_AND_FEED_1 -> setStateFailSafe(RobotState.FEED_1);
            case INTAKE_AND_FEED_2 -> setStateFailSafe(RobotState.FEED_2);

            case PREPARE_INTAKE_AND_SHOOT_HUB -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
            case PREPARE_INTAKE_AND_FEED_1 -> setStateFailSafe(RobotState.PREPARE_FEED_1);
            case PREPARE_INTAKE_AND_FEED_2 -> setStateFailSafe(RobotState.PREPARE_FEED_2);
            default -> setStateFailSafe(RobotState.IDLE);
        }
    }

    public void shootHubWaitRequest() {
        if (getState().intaking) {
            setStateFailSafe(RobotState.WAIT_INTAKE_AND_SHOOT_HUB);
        } else {
            setStateFailSafe(RobotState.WAIT_SHOOT_HUB);
        }
    }

    public void feed1WaitRequest() {
        if (getState().intaking) {
            setStateFailSafe(RobotState.WAIT_INTAKE_AND_FEED_1);
        } else {
            setStateFailSafe(RobotState.WAIT_FEED_1);
        }
    }

    public void feed2WaitRequest() {
        if (getState().intaking) {
            setStateFailSafe(RobotState.WAIT_INTAKE_AND_FEED_2);
        } else {
            setStateFailSafe(RobotState.WAIT_FEED_2);
        }
    }

    public void cancelShotRequest() {
        if (getState().intaking) {
            setStateFailSafe(RobotState.INTAKE);
        } else {
            idleRequest();
        }
    }

    public void confirmShotRequest() {
        switch (getState()) {
            default -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
            case WAIT_SHOOT_HUB -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
            case WAIT_FEED_1 -> setStateFailSafe(RobotState.PREPARE_FEED_1);
            case WAIT_FEED_2 -> setStateFailSafe(RobotState.PREPARE_FEED_2);
            
            case WAIT_INTAKE_AND_SHOOT_HUB -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_SHOOT_HUB);
            case WAIT_INTAKE_AND_FEED_1 -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_FEED_1);
            case WAIT_INTAKE_AND_FEED_2 -> setStateFailSafe(RobotState.PREPARE_INTAKE_AND_FEED_2);
        }
    }

    public void climbSequenceForward() {
        switch (getState()) {
            default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP);
            case CLIMB_1_LINEUP -> setStateFromRequest(RobotState.CLIMB_2_RAISING);
            case CLIMB_2_RAISING -> setStateFromRequest(RobotState.CLIMB_3_HANGING);
            case CLIMB_3_HANGING -> {}
        }
    }

    public void climbSequenceBackward() {
        switch (getState()) {
            default -> {}
            case CLIMB_1_LINEUP -> setStateFromRequest(RobotState.IDLE);
            case CLIMB_2_RAISING -> setStateFromRequest(RobotState.CLIMB_1_LINEUP);
            case CLIMB_3_HANGING -> setStateFromRequest(RobotState.CLIMB_2_RAISING);
        }
    }
}
