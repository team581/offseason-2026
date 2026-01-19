package frc.robot.intake;

public enum IntakeState {
  UNTUNED(0.0),
  UNJAM(-6),

  IDLE(0),
  SHOOTING(12),
  INTAKING(6);

  public final double volts;

  private IntakeState(double volts) {
    this.volts = volts;
  }
}
