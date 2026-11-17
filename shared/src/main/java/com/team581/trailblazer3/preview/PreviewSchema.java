package com.team581.trailblazer3.preview;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Objects.requireNonNull;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * Versioned JSON result schema shared between the warm-JVM stdio worker and the headless Gradle
 * preview runner ({@code ./gradlew previewAuto}).
 */
public final class PreviewSchema {
  /** Build and source identity for reproducibility and cache tracking. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record BuildIdentity(String sourceHash, String classpathHash, long compiledAtEpochMs) {
    public BuildIdentity {
      requireNonNull(sourceHash);
      requireNonNull(classpathHash);
    }
  }

  /** Top-level compile, PathBuilder, or closed-loop runtime diagnostic. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record DiagnosticEntry(
      String severity,
      String code,
      String message,
      @Nullable String segmentId,
      @Nullable String targetGoal,
      @Nullable Double timestampSeconds,
      @Nullable Integer line) {
    public DiagnosticEntry {
      requireNonNull(severity);
      requireNonNull(code);
      requireNonNull(message);
    }
  }

  /** Measured heading arrival error at a waypoint's heading anchor. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record HeadingArrivalEntry(
      String segmentId,
      int goalIndex,
      double errorRadians,
      double errorDegrees,
      boolean withinTolerance) {
    public HeadingArrivalEntry {
      requireNonNull(segmentId);
    }
  }

  /** A convex keepout polygon in Blue-origin field meters. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record KeepoutPolygon(String id, List<PointEntry> vertices) {
    public KeepoutPolygon {
      requireNonNull(id);
      vertices = List.copyOf(vertices);
    }
  }

  /** Hard obstacle polygons (K_hard) and expanded clearance-margin polygons (K_margin). */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record KeepoutSet(
      double marginMeters, List<KeepoutPolygon> hardPolygons, List<KeepoutPolygon> marginPolygons) {
    public KeepoutSet {
      hardPolygons = List.copyOf(hardPolygons);
      marginPolygons = List.copyOf(marginPolygons);
    }
  }

  /** Analytical PathBuilder manifold (line/arc pieces, goal anchors, and sampled planned poses). */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record MathPathEntry(
      double totalLengthMeters,
      List<MathPieceEntry> pieces,
      List<Double> anchors,
      List<SampledMathPose> sampledPoses) {
    public MathPathEntry {
      pieces = List.copyOf(pieces);
      anchors = List.copyOf(anchors);
      sampledPoses = List.copyOf(sampledPoses);
    }
  }

  /** A single analytical line or circular fillet arc produced by {@code PathBuilder}. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record MathPieceEntry(
      String segmentId,
      String type,
      double startProgress,
      double lengthMeters,
      double entryX,
      double entryY,
      double directionX,
      double directionY,
      double curvature) {
    public MathPieceEntry {
      requireNonNull(segmentId);
      requireNonNull(type);
    }
  }

  /** Recorded passage of an authored goal marker during closed-loop simulation. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record PassedMarkerEntry(
      String marker,
      String segmentId,
      double timestampSeconds,
      double progressMeters,
      double x,
      double y) {
    public PassedMarkerEntry {
      requireNonNull(marker);
      requireNonNull(segmentId);
    }
  }

  /** Execution phase durations in milliseconds. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record PhaseTimings(double compileMs, double planMs, double simMs, double totalMs) {}

  /** 2D point in field meters. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record PointEntry(double x, double y) {}

  /** 2D pose in field meters and radians/degrees. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record PoseEntry(double x, double y, double headingRadians, double headingDegrees) {}

  /** Complete versioned preview payload for a single segment or whole autonomous routine. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record PreviewResult(
      int schemaVersion,
      String autoClass,
      String scope,
      String activeSegmentId,
      List<SegmentDescriptor> segments,
      String robotConfigId,
      String alliance,
      long seed,
      double totalDurationSeconds,
      PhaseTimings phaseTimings,
      BuildIdentity buildIdentity,
      RobotFootprint robotFootprint,
      PoseEntry startPose,
      List<WaypointEntry> waypoints,
      MathPathEntry mathPath,
      KeepoutSet keepouts,
      List<TrajectorySample> trajectory,
      List<PassedMarkerEntry> passedMarkers,
      List<HeadingArrivalEntry> headingArrivals,
      String dominantLimitingFactor,
      double minSlowdownRatio,
      List<DiagnosticEntry> diagnostics) {
    public PreviewResult {
      checkArgument(
          schemaVersion == SCHEMA_VERSION,
          "Unsupported schemaVersion %s (expected %s)",
          schemaVersion,
          SCHEMA_VERSION);
      requireNonNull(autoClass);
      requireNonNull(scope);
      requireNonNull(activeSegmentId);
      segments = List.copyOf(segments);
      requireNonNull(robotConfigId);
      requireNonNull(alliance);
      requireNonNull(phaseTimings);
      requireNonNull(buildIdentity);
      requireNonNull(robotFootprint);
      requireNonNull(startPose);
      waypoints = List.copyOf(waypoints);
      requireNonNull(mathPath);
      requireNonNull(keepouts);
      trajectory = List.copyOf(trajectory);
      passedMarkers = List.copyOf(passedMarkers);
      headingArrivals = List.copyOf(headingArrivals);
      requireNonNull(dominantLimitingFactor);
      diagnostics = List.copyOf(diagnostics);
    }
  }

  /**
   * Robot drivebase, 3-inch bumper, and 2026 maximum forward intake extension dimensions in meters.
   */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record RobotFootprint(
      double wheelBaseMeters,
      double trackWidthMeters,
      double halfLengthMeters,
      double halfWidthMeters,
      double bumperLengthMeters,
      double bumperWidthMeters,
      double intakeExtensionMeters,
      double frontIntakeExtentMeters) {}

  /** Sampled pose along the analytical PathBuilder geometry with planned heading. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record SampledMathPose(
      String segmentId,
      double s,
      double x,
      double y,
      double headingRadians,
      double headingDegrees,
      double curvature) {
    public SampledMathPose {
      requireNonNull(segmentId);
    }
  }

  /** Metadata and source spans for one segment in an auto class. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record SegmentDescriptor(
      String id,
      String fieldName,
      int index,
      int declarationStartLine,
      int declarationEndLine,
      List<SourceSpan> relatedSpans) {
    public SegmentDescriptor {
      requireNonNull(id);
      requireNonNull(fieldName);
      relatedSpans = List.copyOf(relatedSpans);
    }
  }

  /** 1-based line range in the Java source file associated with a segment, helper, or state. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record SourceSpan(String kind, String name, int startLine, int endLine) {
    public SourceSpan {
      requireNonNull(kind);
      requireNonNull(name);
    }
  }

  /** Timestamped closed-loop simulation sample with speeds, pace heatmap, and slowdown factor. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record TrajectorySample(
      double timestampSeconds,
      String segmentId,
      int segmentIndex,
      double x,
      double y,
      double headingRadians,
      double headingDegrees,
      double vxMetersPerSec,
      double vyMetersPerSec,
      double omegaRadiansPerSec,
      double commandedSpeedMetersPerSec,
      double achievedSpeedMetersPerSec,
      double maxSpeedMetersPerSec,
      double slowdownRatio,
      double paceSecondsPerMeter,
      double heatmapNormalized,
      double progressMeters,
      String limitingFactor,
      String status,
      String diagnostic) {
    public TrajectorySample {
      requireNonNull(segmentId);
      requireNonNull(limitingFactor);
      requireNonNull(status);
      requireNonNull(diagnostic);
    }
  }

  /** Alliance-transformed waypoint with position, withinMeters radius, heading, and tolerance. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record WaypointEntry(
      int index,
      String segmentId,
      int segmentIndex,
      String id,
      String passType,
      double x,
      double y,
      double withinMeters,
      boolean hasHeadingTarget,
      @Nullable Double targetHeadingRadians,
      @Nullable Double targetHeadingDegrees,
      @Nullable Double headingToleranceRadians,
      @Nullable Double headingToleranceDegrees,
      boolean explicitHeadingTolerance,
      String gatePolicy,
      String turnDirection,
      double anchorOffsetMeters,
      @Nullable String marker,
      double closestPassMeters,
      @Nullable Integer sourceLine,
      @Nullable Integer sourceEndLine) {
    public WaypointEntry {
      requireNonNull(segmentId);
      requireNonNull(id);
      requireNonNull(passType);
      requireNonNull(gatePolicy);
      requireNonNull(turnDirection);
    }
  }

  public static final int SCHEMA_VERSION = 1;

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** Deserializes and validates a {@link PreviewResult} from JSON, rejecting version mismatches. */
  public static PreviewResult fromJson(String json) {
    requireNonNull(json);
    try {
      JsonNode root = MAPPER.readTree(json);
      checkArgument(
          root != null && root.isObject() && root.has("schemaVersion"),
          "Missing schemaVersion in preview payload");
      int version = root.get("schemaVersion").asInt(-1);
      checkArgument(
          version == SCHEMA_VERSION,
          "Unsupported schemaVersion " + version + " (expected " + SCHEMA_VERSION + ")");
      return MAPPER.treeToValue(root, PreviewResult.class);
    } catch (IOException e) {
      throw new IllegalArgumentException("Invalid preview JSON payload", e);
    }
  }

  /** Rounds a finite double to 6 decimal places for deterministic cross-runner output. */
  public static double round6(double value) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return Math.round(value * 1_000_000.0) / 1_000_000.0;
  }

  /** Serializes a {@link PreviewResult} to compact single-line JSON. */
  public static String toJson(PreviewResult result) {
    requireNonNull(result);
    try {
      return MAPPER.writeValueAsString(result);
    } catch (JsonProcessingException e) {
      throw new IllegalStateException("Failed to serialize PreviewResult", e);
    }
  }

  private PreviewSchema() {}
}
