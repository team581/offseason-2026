package frc.robot.robot_manager;

public enum RobotState {
  IDLE,

  WAIT_SCORE,
  PREPARE_SCORE,
  SCORE,

  PREPARE_FORCE_SCORE,
  FORCE_SCORE,

  WAIT_FEED_LEFT,
  PREPARE_FEED_LEFT,
  FEED_LEFT,

  WAIT_FEED_RIGHT,
  PREPARE_FEED_RIGHT,
  FEED_RIGHT,

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

  public boolean isClimbingOrRehoming() {
    return switch (this) {
      case IDLE -> false;
      default -> false;
    };
  }
}
