package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.google.gson.JsonParser;
import com.team581.vision.spot.SpotVisionIO;
import com.team581.vision.spot.VisionIO;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import java.net.ServerSocket;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Real Python publisher to production Java reader, with the offseason robot profile. */
@Tag("spot-transport")
final class SpotPythonTransportTest {
  @TempDir Path temporary;

  @Test
  void readsPythonEstimatesWithTheirPoseAndCaptureTimestamp() throws Exception {
    HAL.initialize(500, 0);
    var repository = Path.of(System.getProperty("spot.repo"));
    var script = repository.resolve("apps/vision/tools/transport_smoke.py");
    assertThat(script).as("Spot checkout containing transport_smoke.py").isRegularFile();
    int port;
    try (var socket = new ServerSocket(0)) {
      port = socket.getLocalPort();
    }
    var output = temporary.resolve("python-publisher.log");
    var profile = VisionConfig.profile();
    try (var server = NetworkTableInstance.create()) {
      server.startServer("", "127.0.0.1", 0, port);
      try (var io = new SpotVisionIO(server, profile, "python-smoke-robot")) {
        io.setTagSelection(
            new VisionIO.TagSelection(
                1, profile.field().tags().stream().map(tag -> tag.id()).sorted().toList()));
        var builder =
            new ProcessBuilder(
                System.getProperty("spot.python"), "-u", script.toString(), "--port", "" + port);
        builder.directory(repository.toFile());
        builder.environment().put("PYTHONPATH", repository.resolve("apps/vision/src").toString());
        builder.redirectErrorStream(true).redirectOutput(output.toFile());
        var publisher = builder.start();
        var received = new ArrayList<VisionIO.Sample>();
        String reason = "No inputs";
        try {
          long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(12);
          while (received.size() < 2 && publisher.isAlive() && System.nanoTime() < deadline) {
            var inputs = io.readInputs(NetworkTablesJNI.now() / 1e6, false);
            reason = inputs.health().reason();
            received.addAll(inputs.samples());
            server.flush();
            Thread.sleep(10);
          }
        } finally {
          publisher.destroy();
          if (!publisher.waitFor(2, TimeUnit.SECONDS)) {
            publisher.destroyForcibly();
            publisher.waitFor(2, TimeUnit.SECONDS);
          }
        }
        var log = Files.readString(output);
        assertThat(received)
            .as("Reader health: %s%nPython output:%n%s", reason, log)
            .hasSizeGreaterThanOrEqualTo(2);
        for (var sample : received) {
          var observation = sample.observation();
          assertThat(observation.cameraName()).isEqualTo(profile.cameras().get(0).name());
          assertThat(observation.robotSession()).isEqualTo("python-smoke-robot");
          assertThat(observation.tagSelectionGeneration()).isEqualTo(1);
          assertThat(observation.candidates()).hasSize(1);
          var pose = observation.candidates().get(0).fieldToCamera();
          assertThat(pose.xM()).isEqualTo(2.25);
          assertThat(pose.yM()).isEqualTo(3.5);
          assertThat(pose.zM()).isEqualTo(.75);
          assertThat(pose.qw()).isEqualTo(1.);
          assertThat(pose.qx()).isEqualTo(0.);
          assertThat(pose.qy()).isEqualTo(0.);
          assertThat(pose.qz()).isEqualTo(0.);
          var sent =
              log.lines()
                  .filter(line -> line.startsWith("{"))
                  .map(line -> JsonParser.parseString(line).getAsJsonObject())
                  .filter(value -> value.get("sequence").getAsLong() == observation.sequence())
                  .findFirst()
                  .orElseThrow();
          assertThat(observation.captureLocalNtUs())
              .isEqualTo(sent.get("capture_local_us").getAsLong());
          assertThat(sample.captureFpgaSeconds())
              .isCloseTo(sent.get("capture_server_us").getAsLong() / 1e6, offset(.000002));
        }
        assertThat(received.get(1).observation().sequence())
            .isGreaterThan(received.get(0).observation().sequence());
        assertThat(received.get(1).captureFpgaSeconds())
            .isGreaterThan(received.get(0).captureFpgaSeconds());
        assertThat(io.malformedCount()).isEqualTo(0);
        System.out.println(
            "PASS: Java read Spot Python estimates at (2.25, 3.5, 0.75) m with matching capture timestamps.");
      }
    }
  }
}
