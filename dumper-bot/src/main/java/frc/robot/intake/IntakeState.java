package frc.robot.intake;
public enum IntakeState {
  UNTUNED(0.0),
  UNJAM(UNTUNED),

  STOPPED(UNTUNED),
  IDLE(UNTUNED),
  INTAKING(UNTUNED),
  OUTTAKING(UNTUNED);

  public final double volts;

  private IntakeState(IntakeState state) {
    this.volts = state.volts;
  }

  private IntakeState(double volts) {
    this.volts = volts;
  }
}

