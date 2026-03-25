package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE_DEPLOYED(true),
  IDLE_STOWED(true),
  INTAKING(true),
  EJECTING(false),
  SHOOT(false),
  SHOOT_AND_INTAKE(false);

  public final boolean canBallFill;

  HopperState(boolean canBallFill) {
    this.canBallFill = canBallFill;
  }
}
