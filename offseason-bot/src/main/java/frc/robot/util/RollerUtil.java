package frc.robot.util;

public final class RollerUtil {
  /**
   * Converts signed motor rotations per second to roller surface velocity in meters per second. The
   * ratio is motor rotations per roller rotation; do not apply Phoenix feedback scaling first.
   */
  public static double surfaceSpeedMetersPerSecond(
      double motorRotationsPerSecond, double rollerDiameterMeters, double motorToRollerRatio) {
    return motorRotationsPerSecond / motorToRollerRatio * Math.PI * rollerDiameterMeters;
  }

  private RollerUtil() {}
}
