package frc.robot.conveyor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ConveyorState {
  SCORE(10),
  FEED(10),
  INITIAL_SCORE(10),
  INITIAL_FEED(10),
  BALL_FILLING(1),
  EJECT(-12),
  IDLE(0),
  INTAKE(2); // ;TBD

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  ConveyorState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Conveyor/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
