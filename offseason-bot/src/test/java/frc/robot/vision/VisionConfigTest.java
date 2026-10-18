package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.data.Offset.offset;

import com.google.common.collect.ImmutableList;
import frc.robot.vision.SpotCameraConfig.Control;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

final class VisionConfigTest {
  @Test
  void defaultsRemainShadowWithoutInventedCameras() {
    var profile = VisionConfig.profile();
    assertThat(profile.getCamerasCount()).isEqualTo(0);
    assertThat(VisionConfig.mounts()).isEmpty();
    assertThat(profile.getFusionEnabled()).isFalse();
    assertThat(profile.getHeadingEnabled()).isFalse();
    assertThat(profile.getField().getTagsCount()).isEqualTo(32);
  }

  @Test
  void invalidCaptureSettingsAndControlsFailBeforePublication() {
    assertThatThrownBy(() -> new CaptureSettings(641, 480, 30, PixelFormat.MJPEG))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new CaptureSettings(640, 480, 0, PixelFormat.MJPEG))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> Control.pinned("gain", Double.NaN))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(
            () ->
                new SpotCameraConfig(
                    "front",
                    true,
                    VisionConfig.mount(0, 0, .3, 0, 0, 0),
                    90,
                    new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
                    ImmutableList.of()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void studentCameraSettingsReachManifestAndOwnTheirLists() {
    var controls =
        new ArrayList<>(
            ImmutableList.of(Control.pinned("exposure_auto", 1), Control.tunable("gain", 5)));
    var camera =
        new SpotCameraConfig(
            "front",
            true,
            VisionConfig.mount(.1, 0, .3, 0, 0, 90),
            180,
            new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
            controls);
    controls.clear();
    var profile = VisionConfig.profile(ImmutableList.of(camera));
    var definition = profile.getCameras(0);
    assertThat(definition.getName()).isEqualTo("front");
    assertThat(definition.getRequired()).isTrue();
    assertThat(definition.getImageRotationDeg()).isEqualTo(180);
    assertThat(definition.getAllowedModesCount()).isEqualTo(1);
    assertThat(definition.getControls(0).getAllowOverride()).isFalse();
    assertThat(definition.getControls(1).getAllowOverride()).isTrue();
    assertThat(SpotProtocol.pose(definition.getRobotToCamera()).getRotation().getZ())
        .isCloseTo(Math.PI / 2, offset(1e-12));
    assertThatThrownBy(() -> VisionConfig.profile(ImmutableList.of(camera, camera)))
        .isInstanceOf(IllegalArgumentException.class);
    assertThat(SpotProtocol.hash(profile))
        .isEqualTo(SpotProtocol.hash(VisionConfig.profile(ImmutableList.of(camera))));
  }
}
