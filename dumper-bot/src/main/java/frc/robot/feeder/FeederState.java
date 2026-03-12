package frc.robot.feeder;

public enum FeederState {
  UNTUNED(0.0),
  UNJAM(-6.0),

  IDLE(0.0),
  FEED(12.0);

  public final double volts;

  FeederState(double volts) {
    this.volts = volts;
  }
}
