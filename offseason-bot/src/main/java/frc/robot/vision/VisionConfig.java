package frc.robot.vision;

import com.team581.util.AprilTags;
import com.team581.vision.spot.SpotProtocol;
import com.team581.vision.spot.data.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Vision settings robot code reads: the field layout and observation limits it shares with Spot,
 * robot-only fusion policy, and the spot-config.json it forwards to Spot without parsing.
 */
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
