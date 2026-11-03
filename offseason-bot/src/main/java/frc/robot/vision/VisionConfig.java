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
  // Fusion is enabled for bringup. Replace ALL placeholder geometry and capture settings first.
  public static final boolean FUSION_ENABLED = true;
  public static final double TAG_SIZE_M = .1651;
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTags.FIELD_LAYOUT;

  public static final List<String> CAMERA_NAMES =
      ImmutableList.of("left", "right", "front", "back");

  // PLACEHOLDERS: cameras at the robot origin, facing the named cardinal direction.
  // These are NOT measured mounts. Verify capture mode and image rotation on each camera.
  public static final SpotCameraConfig LEFT = placeholderCamera("left", 90);
  public static final SpotCameraConfig RIGHT = placeholderCamera("right", -90);
  public static final SpotCameraConfig FRONT = placeholderCamera("front", 0);
  public static final SpotCameraConfig BACK = placeholderCamera("back", 180);
  public static final List<SpotCameraConfig> CAMERAS = ImmutableList.of(LEFT, RIGHT, FRONT, BACK);

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

  private static SpotCameraConfig placeholderCamera(String name, double yawDegrees) {
    return new SpotCameraConfig(
        name,
        true,
        mount(0, 0, 0, 0, 0, yawDegrees),
        0,
        new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
        ImmutableList.of());
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
