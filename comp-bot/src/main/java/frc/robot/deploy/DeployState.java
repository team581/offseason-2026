package frc.robot.deploy;

public enum DeployState {
  INTAKE(0),
  STOWED(0),
  HOMING(0),
  UNHOMED(0);

  public final double angle;

  private DeployState(double angle) {
    this.angle = angle;
  }
}
