package frc.robot.hopper;

public enum HopperState {
  UNTUNED(0.0),
  UNJAM(-6),

  STOPPED(0),
  IDLE(0),
  INTAKING(6),
  OUTTAKING(-6);

  public final double volts;

  private HopperState(double volts) {
    this.volts = volts;
  }
}
