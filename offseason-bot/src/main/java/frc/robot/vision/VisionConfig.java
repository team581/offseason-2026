package frc.robot.vision;

import com.google.common.collect.ImmutableList;
import com.team581.util.AprilTags;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.vision.generated.SpotProtos;
import java.util.HashSet;
import java.util.List;

/** Student-editable vision settings, following the other subsystem Config classes. */
public final class VisionConfig {
  public static final String ROBOT_ID = "offseason-2026";
  // Commissioning gate: enable only after measured mounts and hardware qualification.
  public static final boolean FUSION_ENABLED = false;
  public static final double TAG_SIZE_M = .1651;
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTags.FIELD_LAYOUT;

  public static final List<String> CAMERA_NAMES =
      ImmutableList.of("left", "right", "front", "back");

  // Keep production cameras chassis-fixed and required, in the CAMERA_NAMES order.
  // Add named constants such as FRONT, LEFT, RIGHT and BACK, then list them in CAMERAS.
  // Example shape only: replace the example geometry/capture/control values before enabling.
  // public static final SpotCameraConfig FRONT = new SpotCameraConfig(
  //     "front", true,
  //     mount(.10, 0, .30, 0, 0, 0), // metres, then roll/pitch/yaw in degrees
  //     0, // image rotation: 0 or 180 degrees
  //     new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
  //     List.of(SpotCameraConfig.Control.pinned("exposure_auto", 1)));
  // public static final List<SpotCameraConfig> CAMERAS = List.of(FRONT);
  public static final List<SpotCameraConfig> CAMERAS = ImmutableList.of();

  /** NWU: +X forward, +Y left, +Z up; right-handed roll, pitch, yaw in degrees. */
  public static Transform3d mount(
      double xM, double yM, double zM, double rollDeg, double pitchDeg, double yawDeg) {
    return new Transform3d(
        new Translation3d(xM, yM, zM),
        new Rotation3d(Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
  }

  public static List<SpotCameraProfiles.Camera> mounts() {
    return CAMERAS.stream()
        .map(camera -> SpotCameraProfiles.Camera.fixed(camera.name(), camera.robotToCamera()))
        .toList();
  }

  public static SpotProtos.Profile profile() {
    return profile(CAMERAS);
  }

  static SpotProtos.Profile profile(List<SpotCameraConfig> cameras) {
    SpotCameraConfig.identifier(ROBOT_ID);
    if (!Double.isFinite(TAG_SIZE_M) || TAG_SIZE_M <= 0 || TAG_SIZE_M > 1 || cameras.size() > 32)
      throw new IllegalArgumentException("invalid vision configuration");
    var names = new HashSet<String>();
    for (var camera : cameras) {
      if (!names.add(camera.name())) throw new IllegalArgumentException("duplicate camera name");
    }
    return SpotProtocol.profile(ROBOT_ID, FIELD_LAYOUT, TAG_SIZE_M)
        .addAllCameras(cameras.stream().map(SpotCameraConfig::definition).toList())
        .setFusionEnabled(FUSION_ENABLED)
        .setHeadingEnabled(false)
        .setRevisedTrustEnabled(true)
        .build();
  }

  private VisionConfig() {}
}
