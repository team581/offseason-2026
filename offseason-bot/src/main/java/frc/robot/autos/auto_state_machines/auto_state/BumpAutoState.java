package frc.robot.autos.auto_state_machines.auto_state;

public enum BumpAutoState {
  PREPARE_BUMP_OUT_1,
  CROSS_BUMP_OUT,
  DRIVE_TO_OPPOSITE_HUB,
  WAIT_BEHIND_HUB,
  INTAKE_FIRST_CYCLE,
  CROSS_BUMP_TO_SHOOT_1,
  DRIVE_TO_TOWER,
  SHOOT_1,
  DONE,

  // For when we get beached on a ball
  STUCK_ON_BALL_RECOVERY
}
