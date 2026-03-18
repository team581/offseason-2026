package frc.robot.kicker;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum KickerState {
  SHOOT(12),
  IDLE(0);

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  KickerState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Kicker/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
