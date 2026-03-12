package frc.robot.hopper;

public enum HopperState {
  UNTUNED(0.0),
  UNJAM(-6.0),

  IDLE(0.0),
  INTAKING(12.0),
  SHOOTING(10.0);

  public final double volts;

  HopperState(double volts) {
    this.volts = volts;
  }
}
