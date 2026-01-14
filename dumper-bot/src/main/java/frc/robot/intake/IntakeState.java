package frc.robot.intake;

public enum IntakeState {
  UNTUNED(0.0),
  UNJAM(-6),

  STOPPED(0),
  IDLE(0),
  INTAKING(6),
  OUTTAKING(-6);

  public final double volts;

  private IntakeState(IntakeState state) {
    this.volts = state.volts;
  }

  private IntakeState(double volts) {
    this.volts = volts;
  }
}
