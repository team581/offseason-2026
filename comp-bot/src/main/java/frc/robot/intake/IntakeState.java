package frc.robot.intake;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum IntakeState {
  INTAKE(12),
  EJECT(-12),
  IDLE(0),
  SHOOT(5),
  SHOOT_THEN_INTAKE(3);

  public final double voltage;
  public final DoubleSubscriber tunableVoltage;

  IntakeState(double voltage) {
    this.voltage = voltage;
    this.tunableVoltage = DogLog.tunable("Intake/" + this, voltage);
  }

  public double getVoltage() {
    return tunableVoltage.get();
  }

  public boolean isIntaking() {
    return this == INTAKE || this == SHOOT_THEN_INTAKE;
  }
}
