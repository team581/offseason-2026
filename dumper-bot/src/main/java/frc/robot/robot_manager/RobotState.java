package frc.robot.robot_manager;

public enum RobotState {
    IDLE(false, false, false),

    INTAKE(false, true, false),

    // Shoot Hub
    WAIT_SHOOT_HUB(false, false, false),
    PREPARE_SHOOT_HUB(false, false, false),
    SHOOT_HUB(false, false, true),

    WAIT_INTAKE_AND_SHOOT_HUB(false, true, false),
    PREPARE_INTAKE_AND_SHOOT_HUB(false, true, false),
    INTAKE_AND_SHOOT_HUB(false, true, true),

    //TODO: change feed name
    // Feed 1
    WAIT_FEED_1(false, false, false),
    PREPARE_FEED_1(false, false, false),
    FEED_1(false, false, true),

    WAIT_INTAKE_AND_FEED_1(false, true, false),
    PREPARE_INTAKE_AND_FEED_1(false, true, false),
    INTAKE_AND_FEED_1(false, true, true),

    // Feed 2
    WAIT_FEED_2(false, false, false),
    PREPARE_FEED_2(false, false, false),
    FEED_2(false, false, true),

    WAIT_INTAKE_AND_FEED_2(false, true, false),
    PREPARE_INTAKE_AND_FEED_2(false, true, false),
    INTAKE_AND_FEED_2(false, true, true),

    CLIMB_1_LINEUP(true, false, false),
    CLIMB_2_RAISING(true, false, false),
    CLIMB_3_HANGING(true, false, false);

    public final boolean climbingOrRehoming;
    public final boolean intaking;
    public final boolean shooting;

    private RobotState(boolean climbingOrRehoming, boolean intaking, boolean shooting) {
        this.climbingOrRehoming = climbingOrRehoming;
        this.intaking = intaking;
        this.shooting = shooting;
    }
}
