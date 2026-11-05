package frc.robot.vision;

import com.google.common.collect.ImmutableList;
import com.team581.util.AprilTags;
import com.team581.vision.spot.CaptureSettings;
import com.team581.vision.spot.PixelFormat;
import com.team581.vision.spot.SpotCameraConfig;
import com.team581.vision.spot.SpotProtocol;
import com.team581.vision.spot.data.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.HashSet;
import java.util.List;

/** Student-editable vision settings, following the other subsystem Config classes. */
public final class VisionConfig {
  public static final String ROBOT_ID = "offseason-2026";
  // Fusion is enabled for bringup. Replace ALL placeholder geometry and capture settings first.
  public static final boolean FUSION_ENABLED = true;
  public static final double TAG_SIZE_M = .1651;
  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTags.FIELD_LAYOUT;

  // PLACEHOLDERS: cameras at the robot origin, facing the named cardinal direction.
  // These are NOT measured mounts. Verify capture mode and image rotation on each camera.
  public static final SpotCameraConfig LEFT =
      new SpotCameraConfig(
          "left",
          true,
          mount(
              Units.inchesToMeters(-2.703),
              Units.inchesToMeters(14.972421),
              Units.inchesToMeters(13.369593),
              0,
              -20,
              90),
          0,
          new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
          ImmutableList.of());

  public static final SpotCameraConfig RIGHT =
      new SpotCameraConfig(
          "right",
          true,
          mount(
              Units.inchesToMeters(-6.75),
              Units.inchesToMeters(-14.91),
              Units.inchesToMeters(12.483),
              0,
              -20,
              -90),
          0,
          new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
          ImmutableList.of());
  public static final SpotCameraConfig BACK =
      new SpotCameraConfig(
          "back",
          true,
          mount(
              Units.inchesToMeters(-12.535),
              Units.inchesToMeters(3.250),
              Units.inchesToMeters(13.483),
              0,
              -20,
              180),
          0,
          new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
          ImmutableList.of());
  public static final List<SpotCameraConfig> CAMERAS = ImmutableList.of(LEFT, RIGHT, BACK);

  /** NWU: +X forward, +Y left, +Z up; right-handed roll, pitch, yaw in degrees. */
  public static Transform3d mount(
      double xM, double yM, double zM, double rollDeg, double pitchDeg, double yawDeg) {
    return new Transform3d(
        new Translation3d(xM, yM, zM),
        new Rotation3d(Math.toRadians(rollDeg), Math.toRadians(pitchDeg), Math.toRadians(yawDeg)));
  }

  public static Profile profile() {
    return profile(CAMERAS);
  }

  static Profile profile(List<SpotCameraConfig> cameras) {
    SpotCameraConfig.identifier(ROBOT_ID);
    if (!Double.isFinite(TAG_SIZE_M) || TAG_SIZE_M <= 0 || TAG_SIZE_M > 1 || cameras.size() > 32)
      throw new IllegalArgumentException("invalid vision configuration");
    var names = new HashSet<String>();
    for (var camera : cameras) {
      if (!names.add(camera.name())) throw new IllegalArgumentException("duplicate camera name");
    }
    var defaults = SpotProtocol.profile(ROBOT_ID, FIELD_LAYOUT, TAG_SIZE_M);
    return new Profile(
        ROBOT_ID,
        cameras.stream().map(SpotCameraConfig::definition).toList(),
        defaults.field(),
        FUSION_ENABLED,
        false,
        true,
        defaults.maxAgeS(),
        defaults.futureToleranceS(),
        defaults.reorderWindowS());
  }

  private VisionConfig() {}
}
