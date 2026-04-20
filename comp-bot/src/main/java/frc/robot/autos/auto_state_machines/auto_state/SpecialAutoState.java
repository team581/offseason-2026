package frc.robot.autos.auto_state_machines.auto_state;

public enum SpecialAutoState {
  STARTING_WITH_DELAY,
  INTAKE_ACROSS_MIDLINE,
  DEFAULT_SECOND_INTAKE_SEGMENT,
  DRIVE_BACK_1,
  SHOOT_1,
  DONE,

  // For when we get beached on a ball
  STUCK_ON_BALL_RECOVERY
}
