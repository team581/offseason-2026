package frc.robot.robot_manager;

public enum RobotState {
    IDLE(false),
    INTAKE(false),
    PREPARE_SHOOT_HUB(false),
    SHOOT_HUB(false),
    INTAKE_AND_SHOOT_HUB(false),
    PREPARE_FEED_1(false),
    FEED_1(false),
    INTAKE_AND_FEED_1(false),
    PREPARE_FEED_2(false),
    FEED_2(false),
    INTAKE_AND_FEED_2(false),
    CLIMB_1_LINEUP(true),
    CLIMB_2_RAISING(true),
    CLIMB_3_HANGING(true);

    public final boolean climbingOrRehoming;
    private RobotState(boolean climbingOrRehoming) {
        this.climbingOrRehoming = climbingOrRehoming;
    }
}
