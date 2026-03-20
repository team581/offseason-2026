package frc.robot.robot_manager;

public enum RobotState {
  IDLE,
  // for tuning shooter purposes only
  PREPARE_FORCE_SCORE,
  FORCE_SCORE,

  PREPARE_SCORE,
  SCORE,
  STOP_SHOOTING_SCORE,

  PREPARE_PRESET_SCORE,
  PRESET_SCORE,
  STOP_SHOOTING_PRESET_SCORE,

  PREPARE_PRESET_FEED,
  PRESET_FEED,
  STOP_SHOOTING_PRESET_FEED,

  PREPARE_FEED,
  FEED,
  STOP_SHOOTING_FEED,

  UNJAM,

  CLIMB_1_LINEUP_L1_AUTONOMOUS,
  CLIMB_2_RAISING_L1_AUTONOMOUS,
  CLIMB_3_HANGING_L1_AUTONOMOUS,
  CLIMB_4_RELEASE_L1_AUTONOMOUS,

  AUTOMATIC_CLIMB_1_APPROACH_L1,
  AUTOMATIC_CLIMB_2_LINEUP_L1,
  AUTOMATIC_CLIMB_3_HANGING_L1,

  MANUAL_CLIMB_1_LINEUP_L1,
  MANUAL_CLIMB_2_HANGING_L1;

  public boolean isClimbing() {
    return switch (this) {
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> true;
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> true;
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> true;

      case AUTOMATIC_CLIMB_1_APPROACH_L1 -> true;
      case AUTOMATIC_CLIMB_2_LINEUP_L1 -> true;
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> true;

      case MANUAL_CLIMB_1_LINEUP_L1 -> true;
      case MANUAL_CLIMB_2_HANGING_L1 -> true;

      default -> false;
    };
  }

  public boolean isFeeding() {
    return switch (this) {
      case PREPARE_PRESET_FEED -> true;
      case PRESET_FEED -> true;
      case PREPARE_FEED -> true;
      case FEED -> true;
      default -> false;
    };
  }

  public boolean isScoring() {
    return switch (this) {
      case PREPARE_PRESET_SCORE -> true;
      case PRESET_SCORE -> true;
      case PREPARE_SCORE -> true;
      case SCORE -> true;
      default -> false;
    };
  }
}
