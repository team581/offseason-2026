package frc.robot.robot_manager;

public enum RobotState {
  IDLE,
  // for tuning shooter purposes only
  PREPARE_FORCE_SCORE,
  FORCE_SCORE,

  PREPARE_SCORE,
  SCORE,

  PREPARE_FEED,
  FEED,

  PREPARE_FALLBACK_SCORE,
  FALLBACK_SCORE,

  PREPARE_FALLBACK_FEED,
  FALLBACK_FEED,

  UNJAM;

  public boolean isFeeding() {
    return switch (this) {
      case PREPARE_FALLBACK_FEED, FALLBACK_FEED, PREPARE_FEED, FEED -> true;
      default -> false;
    };
  }

  public boolean isScoring() {
    return switch (this) {
      case PREPARE_FALLBACK_SCORE, FALLBACK_SCORE, PREPARE_SCORE, SCORE -> true;
      default -> false;
    };
  }
}
