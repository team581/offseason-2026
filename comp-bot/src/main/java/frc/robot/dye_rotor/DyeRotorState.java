package frc.robot.dye_rotor;

public enum DyeRotorState {
  IDLE(0.0, 0.0, 0.0),
  WARMUP(0.0, 0.0, 0.0),
  SHOOTING(0.0, 0.0, 0.0),
  UNJAM(0.0, 0.0, 0.0);

  public final double rotorRPM;
  public final double horizontalRPM;
  public final double verticalVoltage;

  private DyeRotorState(double rotorRPM, double horizontalRPM, double verticalVoltage) {
    this.rotorRPM = rotorRPM;
    this.horizontalRPM = horizontalRPM;
    this.verticalVoltage = verticalVoltage;
  }
}
