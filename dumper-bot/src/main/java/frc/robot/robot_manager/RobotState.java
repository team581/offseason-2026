package frc.robot.robot_manager;

public enum RobotState {
  IDLE,

  // Shoot Hub
  WAIT_SHOOT_HUB,
  PREPARE_SHOOT_HUB,
  SHOOT_HUB,

  // TODO: change feed name
  // Feed 1
  WAIT_FEED_1,
  PREPARE_FEED_1,
  FEED_1,

  // Feed 2
  WAIT_FEED_2,
  PREPARE_FEED_2,
  FEED_2,

  CLIMB_1_LINEUP,
  CLIMB_2_RAISING,
  CLIMB_3_HANGING;

  private RobotState() {}

  public boolean climbingOrRehoming() {
    return switch (this) {
      default -> false;
      case CLIMB_1_LINEUP -> true;
      case CLIMB_2_RAISING -> true;
      case CLIMB_3_HANGING -> true;
    };
  }
}
