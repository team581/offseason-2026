package com.team581.trailblazer3.preview;

import static java.nio.charset.StandardCharsets.UTF_8;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.preview.ClasspathManifestManager.ClasspathManifest;
import com.team581.trailblazer3.preview.PreviewRunner.PreviewRequest;
import com.team581.trailblazer3.preview.PreviewSchema.BuildIdentity;
import com.team581.trailblazer3.preview.PreviewSchema.PreviewResult;
import com.team581.trailblazer3.preview.PreviewSchema.SegmentDescriptor;
import com.team581.trailblazer3.preview.PreviewSchema.WaypointEntry;
import com.team581.trailblazer3.preview.WarmPreviewWorker.WorkerResponse;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CancellationException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

final class PreviewTest {
  public static final class DiagnosticCornerAuto {
    public static final Pose2d START = new Pose2d(2.0, 2.0, Rotation2d.kZero);

    public static final AutoSegment SHARP_CORNER_SEGMENT =
        Trailblazer.segment("sharp-corner")
            .through(new Translation2d(4.0, 2.0), 0.30)
            .through(new Translation2d(2.2, 2.25), 0.30)
            .stopAt(new Pose2d(3.0, 2.5, Rotation2d.kZero))
            .build();
  }

  public static final class SampleTwoCycleAuto {
    public enum Marker {
      INTAKE_READY,
      SCORE_READY
    }

    public static final Pose2d START = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0.0));

    public static final AutoSegment CYCLE_1_SEGMENT =
        Trailblazer.segment("cycle-1")
            .through(new Translation2d(3.2, 2.0), 0.35)
            .through(new Pose2d(4.0, 2.6, Rotation2d.fromDegrees(-115.0)), 0.30, 25.0)
            .mark(Marker.INTAKE_READY)
            .stopAt(new Pose2d(4.8, 2.6, Rotation2d.fromDegrees(45.0)))
            .mark(Marker.SCORE_READY)
            .build();

    public static final AutoSegment CYCLE_2_SEGMENT =
        Trailblazer.segment("cycle-2")
            .through(new Translation2d(5.4, 2.2), 0.25)
            .stopAt(new Pose2d(6.0, 2.0, Rotation2d.fromDegrees(90.0)))
            .build();
  }

  private static final ObjectMapper MAPPER = new ObjectMapper();

  @BeforeAll
  static void initHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void classpathManifestCachesInUnder100msAndInvalidatesOnBuildFileChange(@TempDir Path tempDir)
      throws IOException {
    Path buildGradle = tempDir.resolve("build.gradle");
    Path settingsGradle = tempDir.resolve("settings.gradle");
    Files.writeString(buildGradle, "// v1\n", UTF_8);
    Files.writeString(settingsGradle, "rootProject.name = 'test'\n", UTF_8);
    Path dummyClasses = tempDir.resolve("classes");
    Files.createDirectories(dummyClasses);

    ClasspathManifest written =
        ClasspathManifestManager.writeManifest(
            tempDir, ImmutableList.of(dummyClasses.toString()), "", ImmutableList.of());
    assertThat(written.buildFilesHash()).isNotNull();

    long t0 = System.nanoTime();
    Optional<ClasspathManifest> loaded = ClasspathManifestManager.loadIfValid(tempDir);
    double loadMs = (System.nanoTime() - t0) / 1_000_000.0;
    assertThat(loaded.isPresent()).isTrue();
    assertThat(loadMs).isLessThan(100.0);

    // Modifying build.gradle invalidates the cached manifest
    Files.writeString(buildGradle, "// v2 modified\n", UTF_8);
    assertThat(ClasspathManifestManager.loadIfValid(tempDir).isEmpty()).isTrue();
  }

  @Test
  void recordsTopLevelDiagnosticsForNearReversalCorners() {
    var req =
        new PreviewRequest(
            DiagnosticCornerAuto.class.getName(),
            "WHOLE_AUTO",
            null,
            0,
            null,
            "Blue",
            "581",
            581L,
            "",
            0.0,
            new BuildIdentity("src", "cp", 1L));
    PreviewResult res = PreviewRunner.run(DiagnosticCornerAuto.class, req, () -> {});
    assertThat(res.diagnostics())
        .anyMatch(d -> "WARNING".equals(d.severity()) && "NEAR_REVERSAL_CORNER".equals(d.code()));
  }

  @Test
  void schemaRejectsUnsupportedSchemaVersionAndRoundTripsValidPayload() {
    var req =
        new PreviewRequest(
            SampleTwoCycleAuto.class.getName(),
            "SEGMENT",
            "cycle-1",
            0,
            null,
            "Blue",
            "581",
            581L,
            "",
            1.0,
            new BuildIdentity("src-hash", "cp-hash", 1000L));
    PreviewResult result = PreviewRunner.run(SampleTwoCycleAuto.class, req, () -> {});
    String json = PreviewSchema.toJson(result);
    PreviewResult decoded = PreviewSchema.fromJson(json);

    assertEquals(PreviewSchema.SCHEMA_VERSION, decoded.schemaVersion());
    assertEquals("SEGMENT", decoded.scope());
    assertEquals("cycle-1", decoded.activeSegmentId());
    assertEquals("Blue", decoded.alliance());
    assertEquals("581", decoded.robotConfigId());
    assertThat(decoded.totalDurationSeconds() > 0.1).isTrue();
    assertEquals(22.0 * 0.0254, decoded.robotFootprint().wheelBaseMeters(), 1e-5);
    assertEquals(23.5 * 0.0254, decoded.robotFootprint().trackWidthMeters(), 1e-5);
    assertEquals(28.0 * 0.0254, decoded.robotFootprint().bumperLengthMeters(), 1e-5);
    assertEquals(29.5 * 0.0254, decoded.robotFootprint().bumperWidthMeters(), 1e-5);
    assertEquals(11.8 * 0.0254, decoded.robotFootprint().intakeExtensionMeters(), 1e-5);
    assertEquals(25.8 * 0.0254, decoded.robotFootprint().frontIntakeExtentMeters(), 1e-5);

    String badVersionJson = json.replace("\"schemaVersion\":1", "\"schemaVersion\":999");
    IllegalArgumentException ex =
        assertThrows(IllegalArgumentException.class, () -> PreviewSchema.fromJson(badVersionJson));
    assertThat(ex.getMessage()).contains("Unsupported schemaVersion 999");
  }

  @Test
  void secondSegmentSeedsStartPoseFromFirstSegmentTerminalPoseAndRunsUnder50ms() {
    var seg2Req =
        new PreviewRequest(
            SampleTwoCycleAuto.class.getName(),
            "SEGMENT",
            "cycle-2",
            0,
            null,
            "Blue",
            "581",
            581L,
            "",
            0.0,
            new BuildIdentity("src", "cp", 1L));

    long t0 = System.nanoTime();
    PreviewResult seg2 = PreviewRunner.run(SampleTwoCycleAuto.class, seg2Req, () -> {});
    double elapsedMs = (System.nanoTime() - t0) / 1_000_000.0;

    // CYCLE_1_SEGMENT ends at (4.8, 2.6, 45 deg)
    assertEquals(4.8, seg2.startPose().x(), 1e-4);
    assertEquals(2.6, seg2.startPose().y(), 1e-4);
    assertEquals(45.0, seg2.startPose().headingDegrees(), 1e-3);
    assertThat(elapsedMs).isLessThan(50.0);

    // Whole auto runs both segments in sequence across all 3 robot configs
    for (String robotId : ImmutableList.of("581", "9991", "9981")) {
      var wholeReq =
          new PreviewRequest(
              SampleTwoCycleAuto.class.getName(),
              "WHOLE_AUTO",
              null,
              0,
              null,
              "Blue",
              robotId,
              581L,
              "",
              0.0,
              new BuildIdentity("src", "cp", 1L));
      PreviewResult whole = PreviewRunner.run(SampleTwoCycleAuto.class, wholeReq, () -> {});
      assertEquals("WHOLE_AUTO", whole.scope());
      assertEquals(5, whole.waypoints().size());
      assertThat(whole.trajectory()).isNotEmpty();
      assertThat(whole.passedMarkers())
          .extracting(m -> m.marker())
          .contains("INTAKE_READY", "SCORE_READY");
    }
  }

  @Test
  void sourceSpanAnalyzerMapsDeclarationsHelpersAndStatesToSegments() {
    String source =
        """
        package frc.robot.autos;
        public final class ExampleAuto {
          enum State { CYCLE_1, CYCLE_2, DONE }
          static final AutoSegment CYCLE_1_SEGMENT =
              finishCycle(Trailblazer.segment("cycle-1")
                  .through(1.0, 2.0, 0.30));
          static final AutoSegment CYCLE_2_SEGMENT =
              finishCycle(onlyCycle2("cycle-2")
                  .through(2.0, 3.0, 0.30));
          private static AutoSegment finishCycle(AutoSegment.Builder b) {
            return b.stopAt(4.0, 4.0, 0.0).build();
          }
          private static AutoSegment.Builder onlyCycle2(String id) {
            return Trailblazer.segment(id).through(1.5, 2.5, 0.25);
          }
          protected void afterTransition(State s) {
            switch (s) {
              case CYCLE_1 -> {
                drive(CYCLE_1_SEGMENT);
              }
              case CYCLE_2 -> {
                drive(CYCLE_2_SEGMENT);
              }
              case DONE -> stopDriving();
            }
          }
        }
        """;
    List<SegmentDescriptor> descriptors =
        SourceSpanAnalyzer.analyze(
            source,
            ImmutableList.of(
                Map.entry("CYCLE_1_SEGMENT", "cycle-1"), Map.entry("CYCLE_2_SEGMENT", "cycle-2")));
    assertEquals(2, descriptors.size());

    // Line 5 is inside CYCLE_1_SEGMENT declaration
    assertEquals(
        Optional.of("cycle-1"), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 5, null));
    // Line 8 is inside CYCLE_2_SEGMENT declaration
    assertEquals(
        Optional.of("cycle-2"), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 8, null));
    // Line 14 is inside onlyCycle2 helper (only called by CYCLE_2_SEGMENT)
    assertEquals(
        Optional.of("cycle-2"), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 14, null));
    // Line 19 is inside case CYCLE_1 -> drive(CYCLE_1_SEGMENT)
    assertEquals(
        Optional.of("cycle-1"), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 19, null));
    // Line 22 is inside case CYCLE_2 -> drive(CYCLE_2_SEGMENT)
    assertEquals(
        Optional.of("cycle-2"), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 22, null));
    // Line 2 is outside any segment -> empty (defaults to WHOLE_AUTO)
    assertEquals(Optional.empty(), SourceSpanAnalyzer.resolveSegmentAtCursor(descriptors, 2, null));
  }

  @Test
  void warmWorkerOverlayCompilesReloadsEditedStaticSegmentAndHandlesCompileErrorsAndCancellation(
      @TempDir Path tempDir) throws Exception {
    Files.writeString(tempDir.resolve("build.gradle"), "// test", UTF_8);
    String sysCp = System.getProperty("java.class.path", "");
    ClasspathManifestManager.writeManifest(
        tempDir,
        List.of(sysCp.split(java.util.regex.Pattern.quote(java.io.File.pathSeparator))),
        System.getProperty("java.library.path", ""),
        ImmutableList.of());

    WarmPreviewWorker worker = new WarmPreviewWorker(tempDir);
    String sourceV1 =
        """
        package frc.robot.autos;
        import com.team581.trailblazer3.model.AutoSegment;
        import com.team581.trailblazer3.runtime.Trailblazer;
        import edu.wpi.first.math.geometry.Pose2d;
        import edu.wpi.first.math.geometry.Rotation2d;
        public final class DynamicEditAuto {
          public static final Pose2d START = new Pose2d(1.0, 1.0, Rotation2d.kZero);
          public static final AutoSegment SEG =
              Trailblazer.segment("seg-1")
                  .stopAt(3.0, 1.0, 0.0)
                  .build();
        }
        """;

    var paramsV1 = MAPPER.createObjectNode();
    paramsV1.put("sourceContent", sourceV1);
    paramsV1.put("scope", "WHOLE_AUTO");
    paramsV1.put("alliance", "Blue");
    paramsV1.put("robotConfigId", "581");

    WorkerResponse resp1 = worker.executeRequest("req-1", paramsV1, () -> {});
    assertThat(resp1.ok()).isTrue();
    assertThat(resp1.result()).isNotNull();
    assertEquals(3.0, resp1.result().waypoints().get(0).x(), 1e-4);

    // Edit static final AutoSegment stopAt coordinate from 3.0 to 4.25 and warm-refresh
    String sourceV2 = sourceV1.replace(".stopAt(3.0, 1.0, 0.0)", ".stopAt(4.25, 1.0, 0.0)");
    var paramsV2 = MAPPER.createObjectNode();
    paramsV2.put("sourceContent", sourceV2);
    paramsV2.put("scope", "WHOLE_AUTO");
    paramsV2.put("alliance", "Blue");
    paramsV2.put("robotConfigId", "581");

    long warmStart = System.nanoTime();
    WorkerResponse resp2 = worker.executeRequest("req-2", paramsV2, () -> {});
    double warmMs = (System.nanoTime() - warmStart) / 1_000_000.0;
    assertThat(resp2.ok()).isTrue();
    assertThat(resp2.result()).isNotNull();
    assertEquals(4.25, resp2.result().waypoints().get(0).x(), 1e-4);
    assertThat(warmMs).isLessThan(1000.0);

    // Introduce a syntax error -> stale = true, COMPILE_ERROR diagnostic returned
    String brokenSource = sourceV2.replace(".build();", ".build(");
    var paramsBroken = MAPPER.createObjectNode();
    paramsBroken.put("sourceContent", brokenSource);
    WorkerResponse respBroken = worker.executeRequest("req-3", paramsBroken, () -> {});
    assertThat(respBroken.ok()).isFalse();
    assertThat(respBroken.stale()).isTrue();
    assertEquals("COMPILE_ERROR", respBroken.errorCode());
    assertThat(respBroken.diagnostics()).isNotEmpty();

    // Superseded request cancellation
    worker.setLatestRequestId("req-5");
    assertThatThrownBy(() ->
            worker.executeRequest(
                "req-4",
                paramsV2,
                () -> {
                  throw new CancellationException("Superseded");
                }))
        .isInstanceOf(CancellationException.class);
  }

  @Test
  void warmWorkerVerifiesCompilerAndReportsSimulatedJreFailure(@TempDir Path tempDir) {
    assertThat(WarmPreviewWorker.resolveSystemCompiler()).isNotNull();
    System.setProperty("trailblazer.preview.simulateMissingCompiler", "true");
    try {
      assertThat(WarmPreviewWorker.resolveSystemCompiler()).isNull();
      IllegalStateException ex =
          assertThrows(IllegalStateException.class, () -> new WarmPreviewWorker(tempDir));
      assertThat(ex.getMessage()).contains("Java 21 JDK with javac is required");
    } finally {
      System.clearProperty("trailblazer.preview.simulateMissingCompiler");
    }
  }

  @Test
  void waypointTolerancesAndAllianceFlipAreSerializedDeterministically() {
    var blueReq =
        new PreviewRequest(
            SampleTwoCycleAuto.class.getName(),
            "SEGMENT",
            "cycle-1",
            0,
            null,
            "Blue",
            "581",
            581L,
            "",
            0.0,
            new BuildIdentity("src", "cp", 1L));
    var redReq =
        new PreviewRequest(
            SampleTwoCycleAuto.class.getName(),
            "SEGMENT",
            "cycle-1",
            0,
            null,
            "Red",
            "581",
            581L,
            "",
            0.0,
            new BuildIdentity("src", "cp", 1L));

    PreviewResult blue = PreviewRunner.run(SampleTwoCycleAuto.class, blueReq, () -> {});
    PreviewResult red = PreviewRunner.run(SampleTwoCycleAuto.class, redReq, () -> {});

    assertEquals(3, blue.waypoints().size());
    // Waypoint 0: position-only through waypoint
    WaypointEntry wp0 = blue.waypoints().get(0);
    assertEquals(0.35, wp0.withinMeters(), 1e-6);
    assertThat(wp0.hasHeadingTarget()).isFalse();
    assertThat(wp0.targetHeadingDegrees()).isNull();
    assertThat(wp0.headingToleranceDegrees()).isNull();

    // Waypoint 1: through waypoint with within(0.30), heading -115 deg, and headingTolerance(25
    // deg)
    WaypointEntry wp1 = blue.waypoints().get(1);
    assertEquals(0.30, wp1.withinMeters(), 1e-6);
    assertThat(wp1.hasHeadingTarget()).isTrue();
    assertThat(wp1.targetHeadingDegrees()).isNotNull();
    assertEquals(-115.0, wp1.targetHeadingDegrees(), 1e-4);
    assertThat(wp1.headingToleranceDegrees()).isNotNull();
    assertEquals(25.0, wp1.headingToleranceDegrees(), 1e-4);
    assertEquals("INTAKE_READY", wp1.marker());
    assertThat(wp1.closestPassMeters() <= wp1.withinMeters() + 0.05).isTrue();

    // Alliance flip verification
    WaypointEntry redWp1 = red.waypoints().get(1);
    assertEquals(PreviewRunner.FIELD_2026.lengthMeters() - wp1.x(), redWp1.x(), 1e-4);
    assertEquals(PreviewRunner.FIELD_2026.widthMeters() - wp1.y(), redWp1.y(), 1e-4);
    assertThat(redWp1.targetHeadingDegrees()).isNotNull();
    assertEquals(65.0, redWp1.targetHeadingDegrees(), 1e-4);
    assertThat(blue.keepouts().hardPolygons().isEmpty()).isFalse();
    assertThat(blue.keepouts().marginPolygons().isEmpty()).isFalse();
  }
}
