package frc.robot.autos.auto_state_machines.auto_state;

public enum NormalAutoState {
  INTAKE_FIRST_CYCLE,
  DEFAULT_INTAKE_SECOND_CYCLE,
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
