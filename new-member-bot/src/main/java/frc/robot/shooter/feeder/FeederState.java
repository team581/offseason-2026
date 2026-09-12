package frc.robot.shooter.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum FeederState {
  SHOOTING(12), // Shooting positive values can be assigned later
  BALL_FILLING(2), // Ball filling positive values can be assigned later
  INTAKING(0), // Intaking negative value can be assigned later
  EJECT(-2), // Eject negative value
  IDLE(0); // Idle remains 0

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
