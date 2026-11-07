package frc.robot.autos.auto_state_machines.auto_state;

public enum LeftBumpDepotAutoState {
  PREPARE_BUMP_OUT_1,
  CROSS_BUMP_OUT,
  INTAKE_FIRST_CYCLE,
  CROSS_BUMP_TO_SHOOT_1,
  SHOOT_1,
  COLLECT_DEPOT,
  DRIVE_TO_DEPOT_FRONT,
  SHOOT_BACK_TO_HUB,
  DONE,
  STUCK_ON_BALL_RECOVERY
}
