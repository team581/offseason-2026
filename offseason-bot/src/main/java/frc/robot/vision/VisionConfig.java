package frc.robot.vision;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.util.List;

/** The spot.toml robot code forwards to Spot without parsing, and the cameras it defines. */
public final class VisionConfig {
  /** Must match the cameras in spot.toml. */
  public static final List<String> CAMERAS = ImmutableList.of("left", "right", "back");

  public static String spotConfig() {
    var path = Filesystem.getDeployDirectory().toPath().resolve("spot.toml");
    try {
      return Files.readString(path);
    } catch (IOException exception) {
      throw new UncheckedIOException("Cannot read Spot configuration: " + path, exception);
    }
  }

  private VisionConfig() {}
}
