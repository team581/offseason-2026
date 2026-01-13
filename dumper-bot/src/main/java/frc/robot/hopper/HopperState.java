package frc.robot.hopper;

public enum HopperState {
  UNTUNED(0.0),
  UNJAM(UNTUNED),

  STOPPED(UNTUNED),
  IDLE(UNTUNED),
  INTAKING(UNTUNED),
  OUTTAKING(UNTUNED);

  public final double volts;

  private HopperState(HopperState state) {
    this.volts = state.volts;
  }

  private HopperState(double volts) {
    this.volts = volts;
  }
}
