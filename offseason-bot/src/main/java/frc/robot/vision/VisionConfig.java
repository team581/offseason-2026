package frc.robot.vision;

import com.team581.util.AprilTags;
import com.team581.vision.spot.SpotProtocol;
import com.team581.vision.spot.data.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Robot-owned field geometry and fusion policy. */
public final class VisionConfig {
  public static final boolean FUSION_ENABLED = true;
  public static final double TAG_SIZE_M = .1651;
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTags.FIELD_LAYOUT;

  public static String cameraConfig() {
    var path =
        edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("spot-config.json");
    try {
      return java.nio.file.Files.readString(path);
    } catch (java.io.IOException exception) {
      throw new java.io.UncheckedIOException("Cannot read Spot configuration: " + path, exception);
    }
  }

  /** NWU: +X forward, +Y left, +Z up; right-handed roll, pitch, yaw in degrees. */
  public static Transform3d mount(
      double xM, double yM, double zM, double rollDeg, double pitchDeg, double yawDeg) {
    return new Transform3d(
        new Translation3d(xM, yM, zM),
        new Rotation3d(Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
  }

  public static Profile profile() {
    var defaults = SpotProtocol.profile(FIELD_LAYOUT, TAG_SIZE_M);
    return new Profile(
        defaults.cameras(),
        defaults.field(),
        FUSION_ENABLED,
        false,
        defaults.maxAgeS(),
        defaults.futureToleranceS(),
        defaults.reorderWindowS());
  }

  private VisionConfig() {}
}
