package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE,
  INTAKE,
  BALL_FILLING,
  SCORE,
  SCORE_AND_INTAKE,
  REHOME_DEPLOY,

  CLIMB_EMPTY,
  CLIMB_APPROACH,
  CLIMB_LINEUP,
  CLIMB_HANG,

  COMPACT_IN;

  public boolean isIntaking() {
    return this == INTAKE || this == SCORE_AND_INTAKE;
  }

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
