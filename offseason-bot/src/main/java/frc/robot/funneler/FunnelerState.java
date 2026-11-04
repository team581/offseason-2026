package frc.robot.funneler;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum FunnelerState {
  SCORE(0.0),
  BALL_FILLING(0.0),
  IDLE(0.0),
  INTAKE(0.0),
  FEEDING(0.0);

  public final DoubleSubscriber tunableVoltage;

  private FunnelerState(double voltage) {
    this.tunableVoltage = DogLog.tunable("Funneler/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
