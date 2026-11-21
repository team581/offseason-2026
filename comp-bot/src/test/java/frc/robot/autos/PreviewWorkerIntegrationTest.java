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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
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

    var sourceLines = sourceText.lines().map(String::strip).toList();

    var p1 = MAPPER.createObjectNode();
    p1.put("sourceFile", rightTrenchFile.toString());
    p1.put("sourceContent", sourceText);
    p1.put("scope", "CURSOR");
    p1.put("cursorLine", sourceLines.indexOf("static final AutoSegment CYCLE_1_SEGMENT =") + 1);
    p1.put("alliance", "Blue");
    p1.put("robotConfigId", "581");
    WorkerResponse r1 = worker.executeRequest("cur-1", p1, () -> {});
    assertThat(r1.ok()).isTrue();
    assertThat(r1.result()).isNotNull();
    assertEquals("SEGMENT", r1.result().scope());
    assertEquals("right-trench-cycle-1", r1.result().activeSegmentId());

    var p2 = MAPPER.createObjectNode();
    p2.put("sourceFile", rightTrenchFile.toString());
    p2.put("sourceContent", sourceText);
    p2.put("scope", "CURSOR");
    p2.put("cursorLine", sourceLines.indexOf("static final AutoSegment CYCLE_2_SEGMENT =") + 1);
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

    var expectedShoot =
        RightTrenchAuto.shootPose(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
    assertEquals(expectedShoot.getX(), r2.result().startPose().x(), 1e-3);
    assertEquals(expectedShoot.getY(), r2.result().startPose().y(), 1e-3);

    int cycleThreeDriveLine = sourceLines.indexOf("drive(CYCLE_3_SEGMENT);") + 1;
    assertThat(cycleThreeDriveLine).isPositive();
    var p3 = MAPPER.createObjectNode();
    p3.put("sourceFile", rightTrenchFile.toString());
    p3.put("sourceContent", sourceText);
    p3.put("scope", "CURSOR");
    p3.put("cursorLine", cycleThreeDriveLine);
    p3.put("alliance", "Blue");
    p3.put("robotConfigId", "581");
    WorkerResponse r3 = worker.executeRequest("cur-3", p3, () -> {});
    assertThat(r3.ok()).isTrue();
    assertThat(r3.result()).isNotNull();
    assertEquals("SEGMENT", r3.result().scope());
    assertEquals("right-trench-cycle-3", r3.result().activeSegmentId());

    var pWhole = MAPPER.createObjectNode();
    pWhole.put("sourceFile", rightTrenchFile.toString());
    pWhole.put("sourceContent", sourceText);
    pWhole.put("scope", "CURSOR");
    int classLine =
        sourceLines.indexOf(
            "public final class RightTrenchAuto extends TrailblazerAuto<RightTrenchAuto.State> {");
    pWhole.put("cursorLine", classLine + 1);
    pWhole.put("alliance", "Blue");
    pWhole.put("robotConfigId", "581");
    WorkerResponse rWhole = worker.executeRequest("cur-whole", pWhole, () -> {});
    assertThat(rWhole.ok()).isTrue();
    assertThat(rWhole.result()).isNotNull();
    assertEquals("WHOLE_AUTO", rWhole.result().scope());
    assertEquals("WHOLE_AUTO", rWhole.result().activeSegmentId());
  }

  @Test
  void redPreviewUsesRobotWaypointsAndFollowsEdits() throws Exception {
    Path root = resolveRepoRoot();
    Path sourceFile = root.resolve("comp-bot/src/main/java/frc/robot/autos/RightTrenchAuto.java");
    String source = Files.readString(sourceFile);
    var worker = new WarmPreviewWorker(root);
    var request = MAPPER.createObjectNode();
    request.put("sourceFile", sourceFile.toString());
    request.put("scope", "WHOLE_AUTO");
    request.put("alliance", "Red");
    request.put("robotConfigId", "581");
    request.put("sourceContent", source);
    var response = worker.executeRequest("red-full-field", request, () -> {});
    assertThat(response.ok()).withFailMessage("%s", response.errorMessage()).isTrue();
    var preview = response.result();
    assertThat(preview.diagnostics())
        .noneMatch(diagnostic -> diagnostic.severity().equals("ERROR"));
    var robotGoal =
        CompAutoField.segment(RightTrenchAuto.CYCLE_1_SEGMENT, Alliance.Red).goals().get(2);
    assertEquals(robotGoal.pose().getX(), preview.waypoints().get(2).x(), 1e-6);

    int cycleTwoOffset = source.indexOf("Trailblazer.segment(\"right-trench-cycle-2\")");
    int returnOffset = source.indexOf("13.2, RED_TRENCH_Y", cycleTwoOffset);
    int returnLine =
        (int) source.substring(0, returnOffset).chars().filter(c -> c == '\n').count() + 1;
    var returnWaypoint =
        preview.waypoints().stream()
            .filter(waypoint -> waypoint.segmentId().equals("right-trench-cycle-2"))
            .findFirst()
            .orElseThrow();
    assertEquals(returnLine, returnWaypoint.sourceLine());

    request.put(
        "sourceContent",
        source.substring(0, returnOffset)
            + "13.25"
            + source.substring(returnOffset + "13.2".length()));
    var edited = worker.executeRequest("edited", request, () -> {}).result();
    assertEquals(
        returnWaypoint.x() + 0.05, edited.waypoints().get(returnWaypoint.index()).x(), 1e-6);
  }

  @Test
  void workerAndRunnerProduceIdenticalSchemaOutputForTrenchAutosOnBothAlliances() throws Exception {
    Path root = resolveRepoRoot();
    WarmPreviewWorker worker = new WarmPreviewWorker(root);

    for (String autoSimpleName :
        ImmutableList.of("RightTrenchAuto", "LeftTrenchAuto", "TrailblazerTrenchRotationAuto")) {
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
            .withFailMessage("Worker failed: %s", workerResp.errorMessage())
            .isTrue();
        assertThat(elapsedMs).isLessThan(3000.0);
        PreviewResult workerResult = workerResp.result();
        assertThat(workerResult).isNotNull();
        if (autoSimpleName.equals("LeftTrenchAuto")) {
          assertThat(
                  workerResult.waypoints().stream()
                      .map(waypoint -> waypoint.segmentId())
                      .distinct())
              .containsExactly("left-trench-cycle-1", "left-trench-cycle-2", "left-trench-cycle-3");
          assertThat(workerResult.diagnostics())
              .noneMatch(diagnostic -> diagnostic.severity().equals("ERROR"));
        }

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
