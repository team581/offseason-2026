package com.team581.vision.spot;

import edu.wpi.first.math.geometry.Transform3d;

/** Processing mount value type. Each robot owns its camera settings. */
public final class SpotCameraProfiles {
  public record Camera(String name, Transform3d mount, VisionProcessor.Policy qualityPolicy) {
    public static Camera fixed(String name, Transform3d mount) {
      return new Camera(name, mount, null);
    }
  }

  private SpotCameraProfiles() {}
}
