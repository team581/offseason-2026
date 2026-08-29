package frc.robot.robot_manager.hopper_manager;

public enum HopperState {
  IDLE_DEPLOYED(true),
  IDLE_STOWED(false),
  IDLE_SAFE_KICKER_STOW(false),
  INTAKING(true),
  EJECT(false),
  UNJAMMING(false),
  SCORE(false),
  SCORE_AND_INTAKE(false),
  FEED(false),
  FEED_AND_INTAKE(false);

  public final boolean canBallFill;

  HopperState(boolean canBallFill) {
    this.canBallFill = canBallFill;
  }
}
