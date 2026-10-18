package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;

/** Processing mount value type. Student-owned settings live in VisionConfig.java. */
public final class SpotCameraProfiles {
  public record Camera(String name, Transform3d mount, VisionProcessor.Policy qualityPolicy) {
    public static Camera fixed(String name, Transform3d mount) {
      return new Camera(name, mount, null);
    }
  }

  private SpotCameraProfiles() {}
}
