package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.*;

import com.google.common.collect.ImmutableList;
import com.google.gson.JsonParser;
import java.nio.file.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

final class SpotCameraPreviewExportTest {
  @TempDir Path directory;

  @Test
  void exportsComputedMountsAndNormalizedWpilibRotations() throws Exception {
    double[][] rotations = {
      {0, 0, 0}, {90, 0, 0}, {0, 90, 0}, {0, -90, 0}, {0, 0, 90}, {25, -35, 110}
    };
    for (double[] r : rotations) {
      var mount = VisionConfig.mount(.1 + .2, -.4, .5, r[0], r[1], r[2]);
      var camera =
          new SpotCameraConfig(
              "front",
              true,
              mount,
              180,
              new CaptureSettings(640, 480, 30, PixelFormat.MJPEG),
              ImmutableList.of(SpotCameraConfig.Control.tunable("gain", 5)));
      Path output = directory.resolve("profile.json");
      SpotCameraPreviewExport.write(VisionConfig.profile(ImmutableList.of(camera)), output);
      var json = JsonParser.parseString(Files.readString(output)).getAsJsonObject();
      var exported = json.getAsJsonArray("cameras").get(0).getAsJsonObject();
      assertEquals(.3, exported.getAsJsonArray("translation").get(0).getAsDouble(), 1e-12);
      var q = mount.getRotation().getQuaternion();
      double[] expected = {q.getW(), q.getX(), q.getY(), q.getZ()};
      for (int i = 0; i < 4; i++)
        assertEquals(
            expected[i], exported.getAsJsonArray("quaternion").get(i).getAsDouble(), 1e-12);
      assertEquals(180, exported.get("imageRotationDeg").getAsInt());
      assertThat(
              exported
                  .getAsJsonArray("controls")
                  .get(0)
                  .getAsJsonObject()
                  .get("allowOverride")
                  .getAsBoolean())
          .isTrue();
    }
  }

  @Test
  void exportsEmptyRegisteredList() throws Exception {
    Path output = directory.resolve("empty.json");
    SpotCameraPreviewExport.write(VisionConfig.profile(ImmutableList.of()), output);
    assertEquals(
        0,
        JsonParser.parseString(Files.readString(output))
            .getAsJsonObject()
            .getAsJsonArray("cameras")
            .size());
  }
}
