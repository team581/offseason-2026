package frc.robot.funneler;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum FunnelerState {
  // Start with the competition conveyor voltages for the corresponding ball-handling states.
  SCORE(-10.0),
  BALL_FILLING(1.0),
  IDLE(0.0),
  INTAKE(2.0),
  FEEDING(-10.0);

  public final DoubleSubscriber tunableVoltage;

  private FunnelerState(double voltage) {
    this.tunableVoltage = DogLog.tunable("Funneler/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
