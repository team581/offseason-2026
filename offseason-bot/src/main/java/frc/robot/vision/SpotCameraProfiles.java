package frc.robot.vision;

import static com.google.common.base.Preconditions.checkState;

import com.google.common.collect.ImmutableList;
import com.team581.util.AprilTags;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.vision.generated.SpotProtos;
import java.util.List;

/** Offseason Spot settings as code. Add only physically measured camera mounts. */
public final class SpotCameraProfiles {
  // Commissioning gate: measured NWU mounts, verified modes and controls are missing.
  // Disabled definitions still need valid measurements; never publish fake mounts.
  public record Camera(String name, Transform3d mount, VisionProcessor.Policy qualityPolicy) {
    public static Camera fixed(String name, Transform3d mount) {
      return new Camera(name, mount, null);
    }
  }

  public static final boolean FUSION_ENABLED = false;
  public static final List<String> CAMERA_NAMES =
      ImmutableList.of("left", "right", "front", "back");
  public static final List<SpotProtos.CameraDefinition> CAMERAS = ImmutableList.of();
  private static final SpotProtos.Profile PROFILE = createProfile();

  public static List<SpotCameraProfiles.Camera> mounts() {
    checkState(
        CAMERAS.stream().noneMatch(SpotProtos.CameraDefinition::getMovingMount),
        "Moving Spot mount requires an explicit capture-time joint history provider");
    return CAMERAS.stream()
        .map(
            camera ->
                SpotCameraProfiles.Camera.fixed(
                    camera.getName(),
                    new Transform3d(
                        SpotProtocol.pose(camera.getRobotToCamera()).getTranslation(),
                        SpotProtocol.pose(camera.getRobotToCamera()).getRotation())))
        .toList();
  }

  public static SpotProtos.Profile profile() {
    return PROFILE;
  }

  private static SpotProtos.Profile createProfile() {
    return SpotProtocol.profile("offseason-2026", AprilTags.FIELD_LAYOUT, .1651)
        .addAllCameras(CAMERAS)
        .setFusionEnabled(FUSION_ENABLED)
        .setHeadingEnabled(false)
        .setRevisedTrustEnabled(true)
        .build();
  }

  private SpotCameraProfiles() {}
}
