package frc.robot.robot_manager;

public enum RobotState {
  IDLE,

  PREPARE_SCORE,
  SCORE,

  PREPARE_FEED,
  FEED,

  UNJAM,
  REHOME_DEPLOY,
  REHOME_SHOOTER_HOOD,

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
      case CLIMB_1_LINEUP_L1 -> true;
      case CLIMB_1_LINEUP_L1_AUTO -> true;
      case CLIMB_2_RAISING_L1 -> true;
      case CLIMB_2_RAISING_L1_AUTO -> true;
      case CLIMB_3_HANGING_L1 -> true;
      case CLIMB_3_HANGING_L1_AUTO -> true;
      case CLIMB_4_RAISING_L2 -> true;
      case CLIMB_5_HANGING_L2 -> true;
      case CLIMB_6_RAISING_L3 -> true;
      case CLIMB_7_HANGING_L3 -> true;

      case REHOME_DEPLOY -> true;
      case REHOME_SHOOTER_HOOD -> true;

      default -> false;
    };
  }
}
