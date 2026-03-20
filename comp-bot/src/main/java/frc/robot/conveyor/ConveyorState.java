package frc.robot.conveyor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ConveyorState {
  SHOOT(12),
  INTAKE(12),
  IDLE(0);

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
