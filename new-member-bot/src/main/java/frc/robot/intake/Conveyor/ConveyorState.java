package frc.robot.intake.Conveyor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ConveyorState {
  SHOOT(0.0), // TBD
  BALL_FILLING(0.0), // TBD
  EJECT(0.0), // TBD
  IDLE(0.0); // TBD

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  ConveyorState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Conveyor/" + this.name(), voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }
}
