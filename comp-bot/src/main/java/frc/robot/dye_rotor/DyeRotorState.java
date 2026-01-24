package frc.robot.dye_rotor;

public enum DyeRotorState {
  IDLE(0),
  WARMUP(0),
  SHOOTING(0),
  UNJAM(0);

  public final double volts;

  private DyeRotorState(double volts) {
    this.volts = volts;
  }
}
