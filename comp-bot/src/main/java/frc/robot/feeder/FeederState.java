package frc.robot.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum FeederState {
  SHOOT(12),
  INTAKE(12),
  IDLE(0);

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  FeederState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Feeder/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
