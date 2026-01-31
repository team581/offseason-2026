package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(0),
  STOWED(0),
  HOMING(0),
  UNHOMED(0);

  private final double defaultLength;
  private final DoubleSubscriber tunableLength;

  private DeployState(double length) {
    this.defaultLength = length;
    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), length);
  }
  DeployState(DeployState other){
    this(other.defaultLength);
  }

  public double getHeight() {
    return tunableLength.get();
  }
}
