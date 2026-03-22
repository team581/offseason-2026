package frc.robot.robot_manager;

public enum RobotState {
  IDLE,
  // for tuning shooter purposes only
  PREPARE_FORCE_SCORE,
  FORCE_SCORE,

  PREPARE_SCORE,
  SCORE,

  PREPARE_FALLBACK_SCORE,
  FALLBACK_SCORE,

  PREPARE_FALLBACK_FEED,
  FALLBACK_FEED,

  PREPARE_FEED,
  FEED,

  UNJAM;

  public boolean isFeeding() {
    return switch (this) {
      case PREPARE_FALLBACK_FEED -> true;
      case FALLBACK_FEED -> true;
      case PREPARE_FEED -> true;
      case FEED -> true;
      default -> false;
    };
  }

  public boolean isScoring() {
    return switch (this) {
      case PREPARE_FALLBACK_SCORE -> true;
      case FALLBACK_SCORE -> true;
      case PREPARE_SCORE -> true;
      case SCORE -> true;
      default -> false;
    };
  }
}
