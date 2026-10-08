package frc.robot.vision;

import com.team581.util.AprilTags;
import com.team581.vision.VisionCamera;
import com.team581.vision.VisionProfiles;
import com.team581.vision.proto.SpotProtos;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;

/** Offseason Spot settings as code. Add only physically measured camera mounts. */
public final class SpotCameraProfiles {
  // Software integration intentionally starts with no invented Spot hardware.
  // Existing Limelight profiles and ground-camera cluster maps are independent.
  public static final List<SpotProtos.CameraDefinition> CAMERAS = List.of();
  private static final SpotProtos.Profile PROFILE = createProfile();

  public static List<VisionCamera> mounts() {
    if (CAMERAS.stream().anyMatch(SpotProtos.CameraDefinition::getMovingMount)) {
      throw new IllegalStateException(
          "Moving Spot mount requires an explicit capture-time joint history provider");
    }
    return CAMERAS.stream()
        .map(
            camera ->
                VisionCamera.fixed(
                    camera.getName(),
                    new Transform3d(
                        VisionProfiles.pose(camera.getRobotToCamera()).getTranslation(),
                        VisionProfiles.pose(camera.getRobotToCamera()).getRotation())))
        .toList();
  }

  public static SpotProtos.Profile profile() {
    return PROFILE;
  }

  private static SpotProtos.Profile createProfile() {
    return VisionProfiles.profile("offseason-2026", AprilTags.FIELD_LAYOUT, .1651)
        .addAllCameras(CAMERAS)
        .setFusionEnabled(false)
        .setHeadingEnabled(false)
        .setRevisedTrustEnabled(false)
        .build();
  }

  private SpotCameraProfiles() {}
}
