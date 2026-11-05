package frc.robot.vision;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.google.common.collect.ImmutableMap;
import com.google.gson.Gson;
import com.team581.vision.spot.data.CameraDefinition;
import java.nio.file.*;
import java.util.*;

/** Desktop-only export. Never initializes Robot or hardware. */
public final class SpotCameraPreviewExport {
  private record Configuration(List<CameraDefinition> cameras) {}

  public static void main(String[] args) throws Exception {
    var config =
        new ObjectMapper()
            .setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE)
            .disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES)
            .readValue(Path.of(args[1]).toFile(), Configuration.class);
    write(config.cameras(), Path.of(args[0]));
  }

  static void write(List<CameraDefinition> definitions, Path output) throws Exception {
    var cameras = new ArrayList<Object>();
    for (var camera : definitions) {
      var pose = camera.robotToCamera();
      double norm =
          Math.sqrt(
              pose.qw() * pose.qw()
                  + pose.qx() * pose.qx()
                  + pose.qy() * pose.qy()
                  + pose.qz() * pose.qz());
      if (!Double.isFinite(norm) || norm == 0)
        throw new IllegalArgumentException("Invalid quaternion");
      var modes = new ArrayList<Object>();
      for (var mode : camera.allowedModes())
        modes.add(
            ImmutableMap.of(
                "width",
                mode.width(),
                "height",
                mode.height(),
                "fps",
                mode.fps(),
                "pixelFormat",
                mode.pixelFormat()));
      var controls = new ArrayList<Object>();
      for (var control : camera.controls())
        controls.add(
            ImmutableMap.of(
                "name",
                control.name(),
                "value",
                control.defaultValue(),
                "required",
                control.required(),
                "allowOverride",
                control.allowOverride()));
      cameras.add(
          Map.of(
              "name",
              camera.name(),
              "required",
              camera.required(),
              "translation",
              new double[] {pose.xM(), pose.yM(), pose.zM()},
              "quaternion",
              new double[] {pose.qw() / norm, pose.qx() / norm, pose.qy() / norm, pose.qz() / norm},
              "imageRotationDeg",
              camera.imageRotationDeg(),
              "capture",
              modes,
              "controls",
              controls));
    }
    Path target = output.toAbsolutePath();
    Files.createDirectories(target.getParent());
    Path temp = Files.createTempFile(target.getParent(), "preview-", ".json");
    try {
      Files.writeString(
          temp, new Gson().toJson(ImmutableMap.of("schemaVersion", 1, "cameras", cameras)));
      Files.move(temp, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
    } finally {
      Files.deleteIfExists(temp);
    }
  }

  private SpotCameraPreviewExport() {}
}
