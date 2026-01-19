package frc.robot.robot_manager;

public enum RobotState {
  IDLE,

  // Shoot Hub
  WAIT_SHOOT_HUB,
  PREPARE_SHOOT_HUB,
  SHOOT_HUB,

  PREPARE_FORCE_SHOOT,
  FORCE_SHOOT,

  // TODO[@rhetorr]: Change feed state names to be more descriptive
  // Feed 1
  WAIT_FEED_1,
  PREPARE_FEED_1,
  FEED_1,

  // Feed 2
  WAIT_FEED_2,
  PREPARE_FEED_2,
  FEED_2,

  CLIMB_1_LINEUP_L1_AUTO,
  CLIMB_2_RAISING_L1_AUTO,
  CLIMB_3_HANGING_L1_AUTO,

  CLIMB_1_LINEUP_L1,
  CLIMB_2_RAISING_L1,
  CLIMB_3_HANGING_L1,

  CLIMB_4_RAISING_L2,
  CLIMB_5_HANGING_L2,

  CLIMB_6_RAISING_L3,
  CLIMB_7_HANGING_L3;

  private RobotState() {}

  public boolean climbingOrRehoming() {
    return switch (this) {
      default -> false;
      case CLIMB_1_LINEUP_L1_AUTO,
          CLIMB_2_RAISING_L1_AUTO,
          CLIMB_3_HANGING_L1_AUTO,
          CLIMB_1_LINEUP_L1,
          CLIMB_2_RAISING_L1,
          CLIMB_3_HANGING_L1,
          CLIMB_4_RAISING_L2,
          CLIMB_5_HANGING_L2,
          CLIMB_6_RAISING_L3,
          CLIMB_7_HANGING_L3 ->
          true;
    };
  }
}
