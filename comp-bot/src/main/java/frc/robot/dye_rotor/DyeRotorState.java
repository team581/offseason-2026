package frc.robot.dye_rotor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DyeRotorState {
  IDLE(30.0, 0.0, 0.0),
  SHOOT(30.0, 12.0, 12.0),
  UNJAM(0.0, 0.0, 0.0);

  public final double rotorRPM;
  public final double horizontalVoltage;
  public final double verticalVoltage;

  private final DoubleSubscriber verticalVoltageTunable;
  private final DoubleSubscriber horizontalVoltageTunable;

  private DyeRotorState(double rotorRPM, double horizontalVoltage, double verticalVoltage) {
    this.rotorRPM = rotorRPM;
    this.horizontalVoltage = horizontalVoltage;
    this.verticalVoltage = verticalVoltage;

    this.verticalVoltageTunable =
        DogLog.tunable("DyeRotor/VerticalVoltage/" + this, verticalVoltage);
    this.horizontalVoltageTunable =
        DogLog.tunable("DyeRotor/HorizontalVoltage/" + this, horizontalVoltage);
  }

  public double getHorizontalVoltage() {
    return horizontalVoltageTunable.get();
  }

  public double getVerticalVoltage() {
    return verticalVoltageTunable.get();
  }
}
