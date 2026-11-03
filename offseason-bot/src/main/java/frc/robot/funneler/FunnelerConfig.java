package frc.robot.funneler;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;

public class FunnelerConfig {
  // Motor rotations per roller rotation, from the offseason subsystem hardware specs.
  public static final double MOTOR_TO_ROLLER_RATIO = 30.0 / 16.0;

  public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(2.0);

  public static TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
}
