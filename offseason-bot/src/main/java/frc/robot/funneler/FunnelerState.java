package frc.robot.funneler;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum FunnelerState {
  SHOOT(0.0),
  BALL_FILLING(0.0),
  IDLE(123.456),
  INTAKE(581.581);

  public final DoubleSubscriber tunableVoltage;

  private FunnelerState(double voltage) {
    this.tunableVoltage = DogLog.tunable("Conveyor/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
