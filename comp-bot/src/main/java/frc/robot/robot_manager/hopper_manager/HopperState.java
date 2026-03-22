package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE,
  BALL_FILLING,
  SCORE,
  REHOME_DEPLOY,

  CLIMB_EMPTY,
  CLIMB_APPROACH,
  CLIMB_LINEUP,
  CLIMB_HANG;

  public boolean isClimbing() {
    return switch (this) {
      case CLIMB_EMPTY -> true;
      case CLIMB_APPROACH -> true;
      case CLIMB_LINEUP -> true;
      case CLIMB_HANG -> true;
      default -> false;
    };
  }
}
