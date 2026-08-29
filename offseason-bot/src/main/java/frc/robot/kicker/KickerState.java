package frc.robot.kicker;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum KickerState {
  SCORE(0.0),
  FEED(0.0),
  BALL_FILLING(0.0),
  IDLE(0.0),
  INTAKE(0.0);

  public final DoubleSubscriber tunableVoltage;

  private KickerState(double voltage) {
    this.tunableVoltage = DogLog.tunable("Conveyor/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
