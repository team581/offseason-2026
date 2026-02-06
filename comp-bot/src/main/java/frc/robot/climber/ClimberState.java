package frc.robot.climber;

public enum ClimberState {
  STOWED(0),
  L1_LINEUP(1),
  L1_HANGING(1),
  L2_LINEUP(1),
  L2_HANGING(1),
  L3_LINEUP(1),
  L3_HANGING(1);

  public final double height;

  ClimberState(double height) {
    this.height = height;
  }
}
