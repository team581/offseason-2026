package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.preview.PreviewRunner;
import com.team581.trailblazer3.preview.PreviewRunner.PreviewRequest;
import com.team581.trailblazer3.preview.PreviewSchema.PreviewResult;
import com.team581.trailblazer3.preview.WarmPreviewWorker;
import com.team581.trailblazer3.preview.WarmPreviewWorker.WorkerResponse;
import edu.wpi.first.hal.HAL;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class PreviewWorkerIntegrationTest {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  private static Path resolveRepoRoot() {
    Path cwd = Path.of(System.getProperty("user.dir")).toAbsolutePath().normalize();
    if (Files.isRegularFile(cwd.resolve("settings.gradle"))) {
      return cwd;
    }
    if (cwd.getParent() != null && Files.isRegularFile(cwd.resolveSibling("settings.gradle"))) {
      return cwd.getParent();
    }
    return cwd;
  }

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void cursorFollowsSegmentsInRightTrenchAutoAndWarmRefreshesInUnder1Second() throws Exception {
    Path root = resolveRepoRoot();
    Path rightTrenchFile =
        root.resolve("comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java");
    String sourceText = Files.readString(rightTrenchFile);
    WarmPreviewWorker worker = new WarmPreviewWorker(root);

    // Cursor inside CYCLE_1_SEGMENT (line 52)
    var p1 = MAPPER.createObjectNode();
    p1.put("sourceFile", rightTrenchFile.toString());
    p1.put("sourceContent", sourceText);
    p1.put("scope", "CURSOR");
    p1.put("cursorLine", 52);
    p1.put("alliance", "Blue");
    p1.put("robotConfigId", "581");
    WorkerResponse r1 = worker.executeRequest("cur-1", p1, () -> {});
    assertThat(r1.ok()).isTrue();
    assertThat(r1.result()).isNotNull();
    assertEquals("SEGMENT", r1.result().scope());
    assertEquals("right-trench-cycle-1", r1.result().activeSegmentId());

    // Cursor inside CYCLE_2_SEGMENT (line 66) -> warm refresh < 1.0 s and seeded from CYCLE_1 shoot
    // pose
    var p2 = MAPPER.createObjectNode();
    p2.put("sourceFile", rightTrenchFile.toString());
    p2.put("sourceContent", sourceText);
    p2.put("scope", "CURSOR");
    p2.put("cursorLine", 66);
    p2.put("alliance", "Blue");
    p2.put("robotConfigId", "581");
    long warmStart = System.nanoTime();
    WorkerResponse r2 = worker.executeRequest("cur-2", p2, () -> {});
    double warmMs = (System.nanoTime() - warmStart) / 1_000_000.0;
    assertThat(r2.ok()).isTrue();
    assertThat(r2.result()).isNotNull();
    assertEquals("SEGMENT", r2.result().scope());
    assertEquals("right-trench-cycle-2", r2.result().activeSegmentId());
    assertThat(warmMs).isLessThan(1000.0);
    // CYCLE_2_SEGMENT starts at shootPose(Blue), not START
    var expectedShoot =
        RightTrenchAuto.shootPose(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
    assertEquals(expectedShoot.getX(), r2.result().startPose().x(), 1e-3);
    assertEquals(expectedShoot.getY(), r2.result().startPose().y(), 1e-3);

    // Cursor inside case INTAKE_CYCLE_3 -> drive(CYCLE_3_SEGMENT) (line 149)
    var p3 = MAPPER.createObjectNode();
    p3.put("sourceFile", rightTrenchFile.toString());
    p3.put("sourceContent", sourceText);
    p3.put("scope", "CURSOR");
    p3.put("cursorLine", 149);
    p3.put("alliance", "Blue");
    p3.put("robotConfigId", "581");
    WorkerResponse r3 = worker.executeRequest("cur-3", p3, () -> {});
    assertThat(r3.ok()).isTrue();
    assertThat(r3.result()).isNotNull();
    assertEquals("SEGMENT", r3.result().scope());
    assertEquals("right-trench-cycle-3", r3.result().activeSegmentId());

    // Cursor on class header (line 20) -> WHOLE_AUTO
    var pWhole = MAPPER.createObjectNode();
    pWhole.put("sourceFile", rightTrenchFile.toString());
    pWhole.put("sourceContent", sourceText);
    pWhole.put("scope", "CURSOR");
    pWhole.put("cursorLine", 20);
    pWhole.put("alliance", "Blue");
    pWhole.put("robotConfigId", "581");
    WorkerResponse rWhole = worker.executeRequest("cur-whole", pWhole, () -> {});
    assertThat(rWhole.ok()).isTrue();
    assertThat(rWhole.result()).isNotNull();
    assertEquals("WHOLE_AUTO", rWhole.result().scope());
    assertEquals("WHOLE_AUTO", rWhole.result().activeSegmentId());
  }

  @Test
  void workerAndRunnerProduceIdenticalSchemaOutputForRightTrenchAndLeftTrenchOnBothAlliances()
      throws Exception {
    Path root = resolveRepoRoot();
    WarmPreviewWorker worker = new WarmPreviewWorker(root);

    for (String autoSimpleName : ImmutableList.of("RightTrenchAuto", "LeftTrenchAuto")) {
      Path sourceFile =
          root.resolve("comp-bot/src/main/java/frc/robot/autos/" + autoSimpleName + ".java");
      String sourceText = Files.readString(sourceFile);
      Class<?> autoClass = Class.forName("frc.robot.autos." + autoSimpleName);

      for (String alliance : ImmutableList.of("Blue", "Red")) {
        var params = MAPPER.createObjectNode();
        params.put("sourceFile", sourceFile.toString());
        params.put("scope", "WHOLE_AUTO");
        params.put("alliance", alliance);
        params.put("robotConfigId", "581");
        params.put("seed", 581L);

        long t0 = System.nanoTime();
        WorkerResponse workerResp =
            worker.executeRequest("parity-" + autoSimpleName + "-" + alliance, params, () -> {});
        double elapsedMs = (System.nanoTime() - t0) / 1_000_000.0;

        assertThat(workerResp.ok())
            .withFailMessage("Worker failed: " + workerResp.errorMessage())
            .isTrue();
        assertThat(elapsedMs).isLessThan(3000.0);
        PreviewResult workerResult = workerResp.result();
        assertThat(workerResult).isNotNull();

        PreviewRequest directReq =
            new PreviewRequest(
                autoClass.getName(),
                "WHOLE_AUTO",
                null,
                0,
                null,
                alliance,
                "581",
                581L,
                sourceText,
                0.0,
                workerResult.buildIdentity());
        PreviewResult directResult = PreviewRunner.run(autoClass, directReq, () -> {});

        // Verify parity across math path, waypoints, trajectory samples, slowdown factors, and
        // diagnostics
        assertEquals(directResult.schemaVersion(), workerResult.schemaVersion());
        assertEquals(directResult.autoClass(), workerResult.autoClass());
        assertEquals(directResult.scope(), workerResult.scope());
        assertEquals(directResult.alliance(), workerResult.alliance());
        assertEquals(
            directResult.totalDurationSeconds(), workerResult.totalDurationSeconds(), 1e-6);
        assertEquals(directResult.startPose(), workerResult.startPose());
        assertEquals(directResult.waypoints(), workerResult.waypoints());
        assertEquals(directResult.mathPath(), workerResult.mathPath());
        assertEquals(directResult.keepouts(), workerResult.keepouts());
        assertEquals(directResult.trajectory(), workerResult.trajectory());
        assertEquals(directResult.passedMarkers(), workerResult.passedMarkers());
        assertEquals(directResult.headingArrivals(), workerResult.headingArrivals());
        assertEquals(directResult.dominantLimitingFactor(), workerResult.dominantLimitingFactor());
        assertEquals(directResult.minSlowdownRatio(), workerResult.minSlowdownRatio(), 1e-6);
        assertEquals(directResult.diagnostics(), workerResult.diagnostics());
      }
    }
  }
}
