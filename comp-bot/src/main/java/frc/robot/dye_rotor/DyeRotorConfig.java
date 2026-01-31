package frc.robot.dye_rotor;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class DyeRotorConfig {

  public static final int RPM_TOLERANCE_HORIZONTAL = 100;
  public static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("DyeRotor/Horizontal/JamCurrentThreshold", 75.0);

  // TODO: Add all three motor configs here

  // TODO: Move unjam timeout duration here

}
