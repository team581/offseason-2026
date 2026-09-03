package frc.robot.util;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.shooter.ShooterConfig;

public class TiltCompensation {
  private static final DoubleSubscriber HOOD_GAIN =
      DogLog.tunable("TiltCompensation/HoodGain", 1.0);

  /**
   * Hood angle offset (degrees) to compensate for chassis tilt while beached on a ball. Positive =
   * raise the hood. First-order: correction = pitch*cos(shotDir) - roll*sin(shotDir), which for the
   * rear-facing shooter (180°) is simply -pitch.
   */
  public static double getHoodCompensationDegrees(double pitchDegrees, double rollDegrees) {
    var shotDirection = ShooterConfig.SHOOTER_TO_ROBOT.getRotation();
    return HOOD_GAIN.get()
        * (pitchDegrees * shotDirection.getCos() - rollDegrees * shotDirection.getSin());
  }

  private TiltCompensation() {}
}
