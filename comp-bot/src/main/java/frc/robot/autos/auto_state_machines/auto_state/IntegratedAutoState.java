package frc.robot.autos.auto_state_machines.auto_state;

public enum IntegratedAutoState {
  INTAKE_FIRST_CYCLE,
  // Fallback segment for non-cluster map (robot fully on our side)
  DEFAULT_INTAKE_SECOND_CYCLE,

  // Lane 1 for cluster map (robot mostly on our side)
  INTAKE_SECOND_CYCLE_LANE_1,

  // Lane 2 for cluster map (robot mostly on opponent side)
  INTAKE_SECOND_CYCLE_LANE_2,

  // Trench lane for cluster map (trench and then alliance lane)
  INTAKE_SECOND_CYCLE_TRENCH_LANE,

  INTAKE_THIRD_CYCLE,
  CROSS_BUMP_TO_SHOOT_1,
  CROSS_BUMP_TO_SHOOT_2,
  CROSS_BUMP_TO_SHOOT_3,
  SHOOT_1,
  SHOOT_2,
  SHOOT_3,
  DONE,

  // For when we get beached on a ball
  STUCK_ON_BALL_RECOVERY
}
