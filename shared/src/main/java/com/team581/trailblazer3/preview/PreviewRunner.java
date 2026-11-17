package com.team581.trailblazer3.preview;

import static java.util.Objects.requireNonNull;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.math.DrivetrainLimits;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.trailblazer3.model.FieldSymmetry;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import com.team581.trailblazer3.preview.PreviewSchema.BuildIdentity;
import com.team581.trailblazer3.preview.PreviewSchema.DiagnosticEntry;
import com.team581.trailblazer3.preview.PreviewSchema.HeadingArrivalEntry;
import com.team581.trailblazer3.preview.PreviewSchema.KeepoutPolygon;
import com.team581.trailblazer3.preview.PreviewSchema.KeepoutSet;
import com.team581.trailblazer3.preview.PreviewSchema.MathPathEntry;
import com.team581.trailblazer3.preview.PreviewSchema.MathPieceEntry;
import com.team581.trailblazer3.preview.PreviewSchema.PassedMarkerEntry;
import com.team581.trailblazer3.preview.PreviewSchema.PhaseTimings;
import com.team581.trailblazer3.preview.PreviewSchema.PointEntry;
import com.team581.trailblazer3.preview.PreviewSchema.PoseEntry;
import com.team581.trailblazer3.preview.PreviewSchema.PreviewResult;
import com.team581.trailblazer3.preview.PreviewSchema.RobotFootprint;
import com.team581.trailblazer3.preview.PreviewSchema.SampledMathPose;
import com.team581.trailblazer3.preview.PreviewSchema.SegmentDescriptor;
import com.team581.trailblazer3.preview.PreviewSchema.TrajectorySample;
import com.team581.trailblazer3.preview.PreviewSchema.WaypointEntry;
import com.team581.trailblazer3.preview.SourceSpanAnalyzer.WaypointSpan;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;

/**
 * Shared desktop preview runner executing the exact robot {@link PathBuilder} and closed-loop
 * {@link Trailblazer} follower in single-segment or whole-auto mode across {@code Blue}/{@code Red}
 * alliances and robot configurations ({@code 581}, {@code 9991}, {@code 9981}).
 */
public final class PreviewRunner {
  /** Parameters for a single preview execution. */
  public record PreviewRequest(
      String autoClassName,
      String scope,
      @Nullable String segmentId,
      int cursorLine,
      @Nullable String previousSegmentId,
      String alliance,
      String robotConfigId,
      long seed,
      String sourceText,
      double compileMs,
      BuildIdentity buildIdentity) {
    public PreviewRequest {
      requireNonNull(autoClassName);
      requireNonNull(scope);
      requireNonNull(alliance);
      requireNonNull(robotConfigId);
      requireNonNull(sourceText);
      requireNonNull(buildIdentity);
    }
  }

  /**
   * Robot drivetrain, wheel c-c, 3-inch bumper, and 2026 max intake extension configuration for a
   * team number.
   */
  public record RobotProfile(
      String id,
      RobotDrivetrainSpec spec,
      EmpiricalDriveCalibration calibration,
      double wheelBaseMeters,
      double trackWidthMeters,
      double bumperHalfLengthMeters,
      double bumperHalfWidthMeters,
      double intakeExtensionMeters,
      double clearanceHalfSizeMeters) {}

  private record ExtractedAuto(
      Pose2d startPose, List<NamedSegment> segments, List<DiagnosticEntry> diagnostics) {}

  private record NamedSegment(String fieldName, AutoSegment segment, AutoSegment authoredSegment) {
    private NamedSegment(String fieldName, AutoSegment segment) {
      this(fieldName, segment, segment);
    }
  }

  public static final FieldGeometry FIELD_2026 =
      new FieldGeometry(
          FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, FieldSymmetry.ROTATIONAL_180);

  private static final double DT_SECONDS = 0.020;
  private static final double MAX_SEGMENT_SIM_SECONDS = 25.0;
  private static final double CLEARANCE_MARGIN_METERS = 0.04;

  private static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(48.94);
  private static final double TRENCH_HALF_DEPTH_METERS = Units.inchesToMeters(47.0 / 2.0);
  private static final double TRENCH_BUMP_BARRIER_WIDTH_METERS = Units.inchesToMeters(12.0);
  private static final double HUB_HALF_WIDTH_METERS = Units.inchesToMeters(47.0 / 2.0);
  private static final double HUB_NET_EXTENSION_X_METERS = Units.inchesToMeters(10.26);
  private static final double HUB_NET_HALF_WIDTH_Y_METERS = Units.inchesToMeters(58.41 / 2.0);
  private static final double BLUE_CLIMB_MAX_X_METERS =
      FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(608.375814);
  private static final double BLUE_CLIMB_MIN_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(193.718750);
  private static final double BLUE_CLIMB_MAX_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(146.718750);

  private static final List<String> KEEPOUT_NAMES =
      List.of(
          "blue-right-barrier",
          "blue-left-barrier",
          "blue-hub",
          "blue-climb-tower",
          "red-right-barrier",
          "red-left-barrier",
          "red-hub",
          "red-climb-tower");

  /** Builds the 2026 field clearance model for the given robot profile and alliance. */
  public static FieldClearance buildClearance(RobotProfile profile, Alliance alliance) {
    double blueObstacleX = FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getX();
    double blueNear = blueObstacleX - TRENCH_HALF_DEPTH_METERS;
    double blueFar = blueObstacleX + TRENCH_HALF_DEPTH_METERS;
    double rightBarrierLowY = TRENCH_WIDTH_METERS;
    double rightBarrierHighY = TRENCH_WIDTH_METERS + TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double leftBarrierHighY = FIELD_2026.widthMeters() - TRENCH_WIDTH_METERS;
    double leftBarrierLowY = leftBarrierHighY - TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double hubCenterY = FIELD_2026.widthMeters() / 2.0;

    var blueRightBarrier = rectangle(blueNear, rightBarrierLowY, blueFar, rightBarrierHighY);
    var blueLeftBarrier = rectangle(blueNear, leftBarrierLowY, blueFar, leftBarrierHighY);
    var blueHub =
        ImmutableList.of(
            new Translation2d(
                blueObstacleX - HUB_HALF_WIDTH_METERS, hubCenterY - HUB_HALF_WIDTH_METERS),
            new Translation2d(
                blueObstacleX + HUB_HALF_WIDTH_METERS + HUB_NET_EXTENSION_X_METERS,
                hubCenterY - HUB_NET_HALF_WIDTH_Y_METERS),
            new Translation2d(
                blueObstacleX + HUB_HALF_WIDTH_METERS + HUB_NET_EXTENSION_X_METERS,
                hubCenterY + HUB_NET_HALF_WIDTH_Y_METERS),
            new Translation2d(
                blueObstacleX - HUB_HALF_WIDTH_METERS, hubCenterY + HUB_HALF_WIDTH_METERS));
    var blueClimbTower =
        rectangle(0.0, BLUE_CLIMB_MIN_Y_METERS, BLUE_CLIMB_MAX_X_METERS, BLUE_CLIMB_MAX_Y_METERS);

    var blueClearance =
        new FieldClearance(
            FIELD_2026,
            profile.clearanceHalfSizeMeters(),
            profile.clearanceHalfSizeMeters(),
            CLEARANCE_MARGIN_METERS,
            ImmutableList.of(
                blueRightBarrier,
                blueLeftBarrier,
                blueHub,
                blueClimbTower,
                flipPolygon(blueRightBarrier),
                flipPolygon(blueLeftBarrier),
                flipPolygon(blueHub),
                flipPolygon(blueClimbTower)));
    return blueClearance.forAlliance(alliance);
  }

  /** Returns the {@link RobotProfile} for {@code 581}, {@code 9991}, or {@code 9981}. */
  public static RobotProfile robotProfile(String robotConfigId) {
    double bumperSideThickness = Units.inchesToMeters(3.0);
    double maxIntakeExt2026 = Units.inchesToMeters(11.8);
    double clearanceHalfSize = Units.inchesToMeters(35.0 / 2.0);
    return switch (robotConfigId == null ? "581" : robotConfigId.trim()) {
      case "9991" -> {
        double modX = Units.inchesToMeters(10.0);
        double modY = Units.inchesToMeters(12.0);
        var spec =
            new RobotDrivetrainSpec(
                59.0,
                4.95,
                Units.inchesToMeters(2.0),
                7.846153846153847,
                11.314285714285717,
                ImmutableList.of(
                    new Translation2d(modX, modY),
                    new Translation2d(modX, -modY),
                    new Translation2d(-modX, modY),
                    new Translation2d(-modX, -modY)),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX44Foc(1),
                120.0,
                0.20);
        yield new RobotProfile(
            "9991",
            spec,
            new EmpiricalDriveCalibration(8.5, 6.0, 0.055),
            modX * 2.0,
            modY * 2.0,
            modX + bumperSideThickness,
            modY + bumperSideThickness,
            maxIntakeExt2026,
            clearanceHalfSize);
      }
      case "9981" -> {
        double modX = Units.inchesToMeters(11.0);
        double modY = Units.inchesToMeters(11.75);
        var spec =
            new RobotDrivetrainSpec(
                56.0,
                4.80,
                Units.inchesToMeters(2.0),
                7.673684210526316,
                12.1,
                ImmutableList.of(
                    new Translation2d(modX, modY),
                    new Translation2d(modX, -modY),
                    new Translation2d(-modX, modY),
                    new Translation2d(-modX, -modY)),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX44(1),
                90.0,
                0.20);
        yield new RobotProfile(
            "9981",
            spec,
            new EmpiricalDriveCalibration(8.0, 5.5, 0.060),
            modX * 2.0,
            modY * 2.0,
            modX + bumperSideThickness,
            modY + bumperSideThickness,
            maxIntakeExt2026,
            clearanceHalfSize);
      }
      default -> {
        double modX = Units.inchesToMeters(11.0);
        double modY = Units.inchesToMeters(11.75);
        var spec =
            new RobotDrivetrainSpec(
                61.2,
                5.20,
                Units.inchesToMeters(2.0),
                7.673684210526316,
                12.1,
                ImmutableList.of(
                    new Translation2d(modX, modY),
                    new Translation2d(modX, -modY),
                    new Translation2d(-modX, modY),
                    new Translation2d(-modX, -modY)),
                DCMotor.getKrakenX60Foc(1),
                DCMotor.getKrakenX44Foc(1),
                120.0,
                0.20);
        yield new RobotProfile(
            "581",
            spec,
            new EmpiricalDriveCalibration(8.5, 6.0, 0.055),
            modX * 2.0,
            modY * 2.0,
            modX + bumperSideThickness,
            modY + bumperSideThickness,
            maxIntakeExt2026,
            clearanceHalfSize);
      }
    };
  }

  /** Executes a preview request against a loaded auto class and returns a {@link PreviewResult}. */
  public static PreviewResult run(
      Class<?> autoClass, PreviewRequest request, Runnable checkCancelled) {
    requireNonNull(autoClass);
    requireNonNull(request);
    requireNonNull(checkCancelled);

    long startNs = System.nanoTime();
    checkCancelled.run();

    Alliance alliance = "Red".equalsIgnoreCase(request.alliance()) ? Alliance.Red : Alliance.Blue;
    String allianceName = alliance == Alliance.Red ? "Red" : "Blue";
    RobotProfile profile = robotProfile(request.robotConfigId());
    FieldClearance clearance = buildClearance(profile, alliance);

    var capturedLines = new IdentityHashMap<AutoSegment, List<Integer>>();
    ExtractedAuto extracted =
        AutoSegment.Builder.captureGoalSources(
            autoClass.getName(), capturedLines::put, () -> extractAuto(autoClass, profile));
    List<DiagnosticEntry> diagnostics = new ArrayList<>(extracted.diagnostics());

    List<Map.Entry<String, String>> knownPairs = new ArrayList<>();
    for (NamedSegment ns : extracted.segments()) {
      knownPairs.add(Map.entry(ns.fieldName(), ns.segment().id()));
    }
    List<SegmentDescriptor> segmentDescriptors =
        SourceSpanAnalyzer.analyze(request.sourceText(), knownPairs);

    // An auto can mix already-initialized segments with segments built during extraction.
    Map<String, List<Integer>> waypointLinesByField = new LinkedHashMap<>();
    if (extracted.segments().stream()
        .anyMatch(segment -> !capturedLines.containsKey(segment.authoredSegment()))) {
      for (var rawSeg : SourceSpanAnalyzer.parseRawSegments(request.sourceText())) {
        waypointLinesByField.put(rawSeg.fieldName(), rawSeg.waypointLines());
      }
    }

    var waypointSpans = SourceSpanAnalyzer.waypointSpans(request.sourceText());

    // Resolve scope and activeSegmentId
    String requestedScope = request.scope().toUpperCase(Locale.ROOT);
    String targetSegmentId = request.segmentId();
    if (!"WHOLE_AUTO".equals(requestedScope)) {
      if ((targetSegmentId == null || targetSegmentId.isBlank()) && request.cursorLine() > 0) {
        targetSegmentId =
            SourceSpanAnalyzer.resolveSegmentAtCursor(
                    segmentDescriptors, request.cursorLine(), request.previousSegmentId())
                .orElse(null);
      }
    }

    int targetSegmentIndex = -1;
    if (!"WHOLE_AUTO".equals(requestedScope)
        && targetSegmentId != null
        && !targetSegmentId.isBlank()) {
      for (int i = 0; i < extracted.segments().size(); i++) {
        NamedSegment ns = extracted.segments().get(i);
        if (ns.segment().id().equals(targetSegmentId) || ns.fieldName().equals(targetSegmentId)) {
          targetSegmentIndex = i;
          break;
        }
      }
    }

    String effectiveScope = targetSegmentIndex >= 0 ? "SEGMENT" : "WHOLE_AUTO";
    String activeSegmentId =
        targetSegmentIndex >= 0
            ? extracted.segments().get(targetSegmentIndex).segment().id()
            : "WHOLE_AUTO";

    Pose2d allianceAutoStart =
        alliance == Alliance.Blue
            ? extracted.startPose()
            : FIELD_2026.flipPose(extracted.startPose());

    // Determine which segments to execute and seed initial pose
    Pose2d entryPose = allianceAutoStart;
    int firstSegIdx = 0;
    int lastSegIdx = extracted.segments().size() - 1;
    if (targetSegmentIndex >= 0) {
      firstSegIdx = targetSegmentIndex;
      lastSegIdx = targetSegmentIndex;
      // Chain preceding segments to seed segment N's starting pose from segment N-1's end pose
      for (int i = 0; i < targetSegmentIndex; i++) {
        checkCancelled.run();
        var prior = extracted.segments().get(i).segment();
        AutoSegment priorSeg = prior.forAlliance(alliance, FIELD_2026);
        try {
          var priorPath =
              PathBuilder.build(
                  entryPose.getTranslation(),
                  priorSeg.goals(),
                  profile.calibration().maxBrakingDeceleration(),
                  clearance);
          Pose2d[] sampled =
              Trailblazer.sampleSegmentPoses(entryPose, priorPath, priorSeg.goals(), 0.10);
          entryPose =
              sampled.length > 0
                  ? sampled[sampled.length - 1]
                  : terminalGoalPose(priorSeg.goals().get(priorSeg.goals().size() - 1), entryPose);
        } catch (RuntimeException ignored) {
          var goals = priorSeg.goals();
          entryPose = terminalGoalPose(goals.get(goals.size() - 1), entryPose);
        }
      }
    }

    long planStartNs = System.nanoTime();
    List<MathPieceEntry> mathPieces = new ArrayList<>();
    List<Double> mathAnchors = new ArrayList<>();
    List<SampledMathPose> sampledMathPoses = new ArrayList<>();
    List<WaypointEntry> waypoints = new ArrayList<>();
    List<TrajectorySample> trajectory = new ArrayList<>();
    List<PassedMarkerEntry> passedMarkers = new ArrayList<>();
    List<HeadingArrivalEntry> headingArrivals = new ArrayList<>();
    EnumMap<LimitingFactor, Integer> limiterCounts = new EnumMap<>(LimitingFactor.class);
    double minSlowdownRatio = 1.0;

    double cumulativeLength = 0.0;
    double simTime = 0.0;
    Pose2d currentPose = entryPose;
    ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    double planMsAccum = 0.0;
    double simMsAccum = 0.0;
    int globalWaypointIndex = 0;

    for (int segIdx = firstSegIdx; segIdx <= lastSegIdx; segIdx++) {
      checkCancelled.run();
      NamedSegment namedSeg = extracted.segments().get(segIdx);
      AutoSegment segment = namedSeg.segment().forAlliance(alliance, FIELD_2026);
      String segId = segment.id();
      List<Goal> goals = segment.goals();
      var captured = capturedLines.get(namedSeg.authoredSegment());
      List<Integer> wpLines =
          captured != null
              ? captured
              : waypointLinesByField.getOrDefault(namedSeg.fieldName(), ImmutableList.of());
      if (wpLines.size() != goals.size()) {
        wpLines = ImmutableList.of();
      }
      Integer segDeclLine =
          segIdx < segmentDescriptors.size()
                  && segmentDescriptors.get(segIdx).declarationStartLine() > 0
              ? segmentDescriptors.get(segIdx).declarationStartLine()
              : null;

      long segPlanStart = System.nanoTime();
      PathBuilder.Path builtPath;
      try {
        builtPath =
            PathBuilder.build(
                currentPose.getTranslation(),
                goals,
                profile.calibration().maxBrakingDeceleration(),
                clearance);
      } catch (RuntimeException ex) {
        diagnostics.add(
            new DiagnosticEntry(
                "ERROR",
                "PATH_BUILD_ERROR",
                ex.getMessage() != null ? ex.getMessage() : "PathBuilder failed",
                segId,
                null,
                null,
                segDeclLine));
        break;
      }

      for (String warning : builtPath.warnings()) {
        String code = warning;
        String targetGoal = null;
        int spaceIdx = warning.indexOf(' ');
        if (spaceIdx > 0) {
          code = warning.substring(0, spaceIdx);
          targetGoal = warning.substring(spaceIdx + 1).trim();
        }
        Integer line = segDeclLine;
        if (targetGoal != null && targetGoal.startsWith("goal-")) {
          try {
            int gIdx = Integer.parseInt(targetGoal.substring("goal-".length()));
            if (gIdx >= 0 && gIdx < wpLines.size()) {
              line = wpLines.get(gIdx) > 0 ? wpLines.get(gIdx) : segDeclLine;
            }
          } catch (NumberFormatException ignored) {
            // keep segDeclLine
          }
        }
        diagnostics.add(
            new DiagnosticEntry("WARNING", code, warning, segId, targetGoal, null, line));
      }

      for (PathBuilder.Piece piece : builtPath.pieces()) {
        mathPieces.add(
            new MathPieceEntry(
                segId,
                Math.abs(piece.curvature()) < 1e-9 ? "LINE" : "ARC",
                PreviewSchema.round6(cumulativeLength + piece.start()),
                PreviewSchema.round6(piece.length()),
                PreviewSchema.round6(piece.entry().getX()),
                PreviewSchema.round6(piece.entry().getY()),
                PreviewSchema.round6(piece.direction().getX()),
                PreviewSchema.round6(piece.direction().getY()),
                PreviewSchema.round6(piece.curvature())));
      }
      for (Double anchor : builtPath.anchors()) {
        mathAnchors.add(PreviewSchema.round6(cumulativeLength + anchor));
      }

      Pose2d[] sampled = Trailblazer.sampleSegmentPoses(currentPose, builtPath, goals, 0.05);
      int sampleCount = sampled.length;
      for (int i = 0; i < sampleCount; i++) {
        if (segIdx > firstSegIdx && i == 0 && !sampledMathPoses.isEmpty()) {
          continue;
        }
        double localS = sampleCount > 1 ? (builtPath.length() * i) / (sampleCount - 1) : 0.0;
        Pose2d p = sampled[i];
        sampledMathPoses.add(
            new SampledMathPose(
                segId,
                PreviewSchema.round6(cumulativeLength + localS),
                PreviewSchema.round6(p.getX()),
                PreviewSchema.round6(p.getY()),
                PreviewSchema.round6(p.getRotation().getRadians()),
                PreviewSchema.round6(p.getRotation().getDegrees()),
                PreviewSchema.round6(builtPath.curvatureAt(localS))));
      }
      planMsAccum += (System.nanoTime() - segPlanStart) / 1_000_000.0;

      // Closed-loop simulation of this segment
      long segSimStart = System.nanoTime();
      double[] clock = {simTime};
      DoubleSupplier clockSupplier = () -> clock[0];
      Trailblazer follower =
          new Trailblazer(
              clockSupplier,
              profile.spec(),
              profile.calibration(),
              DT_SECONDS,
              Optional.of(clearance));
      follower.activate(segment, clearance);

      DrivetrainLimits derivedLimits =
          DrivetrainLimits.derive(
              profile.spec(), profile.calibration(), DT_SECONDS, segment.limits());
      double maxSpeed = Math.max(0.1, derivedLimits.maxSpeed());

      double[] closestPass = new double[goals.size()];
      for (int g = 0; g < goals.size(); g++) {
        closestPass[g] =
            currentPose.getTranslation().getDistance(goals.get(g).pose().getTranslation());
      }
      Set<Enum<?>> recordedMarkers = new LinkedHashSet<>();
      Set<String> seenDiagnosticsInSeg = new LinkedHashSet<>();

      double segElapsed = 0.0;
      double responseAlpha =
          -Math.expm1(-DT_SECONDS / profile.calibration().driveResponseLagSeconds());

      while (segElapsed <= MAX_SEGMENT_SIM_SECONDS) {
        checkCancelled.run();
        clock[0] = simTime;
        TickResult tick = follower.update(simTime, currentPose, currentSpeeds);
        var cmd = tick.commandedFieldSpeeds();
        double cmdSpeed = Math.hypot(cmd.vxMetersPerSecond, cmd.vyMetersPerSecond);
        double achSpeed =
            Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        double slowdownRatio = MathUtil.clamp(cmdSpeed / maxSpeed, 0.0, 1.0);
        if (tick.status() == TickResult.RuntimeStatus.RUNNING && cmdSpeed > 0.05) {
          minSlowdownRatio = Math.min(minSlowdownRatio, slowdownRatio);
        }
        if (tick.limitingFactor() != LimitingFactor.NONE
            && tick.limitingFactor() != LimitingFactor.TOP_SPEED) {
          limiterCounts.merge(tick.limitingFactor(), 1, Integer::sum);
        }

        double pace = 1.0 / Math.max(0.15, achSpeed);
        double heatmapNorm = MathUtil.clamp(achSpeed / maxSpeed, 0.0, 1.0);

        trajectory.add(
            new TrajectorySample(
                PreviewSchema.round6(simTime),
                segId,
                segIdx,
                PreviewSchema.round6(currentPose.getX()),
                PreviewSchema.round6(currentPose.getY()),
                PreviewSchema.round6(currentPose.getRotation().getRadians()),
                PreviewSchema.round6(currentPose.getRotation().getDegrees()),
                PreviewSchema.round6(currentSpeeds.vxMetersPerSecond),
                PreviewSchema.round6(currentSpeeds.vyMetersPerSecond),
                PreviewSchema.round6(currentSpeeds.omegaRadiansPerSecond),
                PreviewSchema.round6(cmdSpeed),
                PreviewSchema.round6(achSpeed),
                PreviewSchema.round6(maxSpeed),
                PreviewSchema.round6(slowdownRatio),
                PreviewSchema.round6(pace),
                PreviewSchema.round6(heatmapNorm),
                PreviewSchema.round6(cumulativeLength + tick.progress()),
                tick.limitingFactor().name(),
                tick.status().name(),
                tick.diagnostic()));

        for (int g = 0; g < goals.size(); g++) {
          Goal goal = goals.get(g);
          double d = currentPose.getTranslation().getDistance(goal.pose().getTranslation());
          if (d < closestPass[g]) {
            closestPass[g] = d;
          }
          if (goal.marker() != null
              && !recordedMarkers.contains(goal.marker())
              && follower.passedMarker(goal.marker())) {
            recordedMarkers.add(goal.marker());
            passedMarkers.add(
                new PassedMarkerEntry(
                    goal.marker().name(),
                    segId,
                    PreviewSchema.round6(simTime),
                    PreviewSchema.round6(cumulativeLength + tick.progress()),
                    PreviewSchema.round6(currentPose.getX()),
                    PreviewSchema.round6(currentPose.getY())));
          }
        }

        if (!tick.diagnostic().isEmpty() && seenDiagnosticsInSeg.add(tick.diagnostic())) {
          String diagCode = tick.diagnostic();
          String sev =
              "KEEPOUT_VIOLATION".equals(diagCode) || "NONFINITE_STATE".equals(diagCode)
                  ? "ERROR"
                  : "WARNING";
          diagnostics.add(
              new DiagnosticEntry(
                  sev,
                  diagCode,
                  diagCode + " @ " + "%.2fs".formatted(simTime),
                  segId,
                  null,
                  PreviewSchema.round6(simTime),
                  segDeclLine));
        }
        if (tick.infeasible() && seenDiagnosticsInSeg.add("INFEASIBLE")) {
          diagnostics.add(
              new DiagnosticEntry(
                  "WARNING",
                  "INFEASIBLE",
                  "INFEASIBLE @ " + "%.2fs".formatted(simTime),
                  segId,
                  null,
                  PreviewSchema.round6(simTime),
                  segDeclLine));
        }

        if (follower.atGoal()) {
          break;
        }

        currentSpeeds =
            new ChassisSpeeds(
                currentSpeeds.vxMetersPerSecond
                    + responseAlpha * (cmd.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond),
                currentSpeeds.vyMetersPerSecond
                    + responseAlpha * (cmd.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond),
                currentSpeeds.omegaRadiansPerSecond
                    + responseAlpha
                        * (cmd.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond));
        currentPose =
            new Pose2d(
                currentPose.getX() + currentSpeeds.vxMetersPerSecond * DT_SECONDS,
                currentPose.getY() + currentSpeeds.vyMetersPerSecond * DT_SECONDS,
                currentPose
                    .getRotation()
                    .plus(
                        Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * DT_SECONDS)));
        simTime += DT_SECONDS;
        segElapsed += DT_SECONDS;
      }

      for (Trailblazer.HeadingArrival arrival : follower.headingArrivals()) {
        headingArrivals.add(
            new HeadingArrivalEntry(
                segId,
                arrival.goalIndex(),
                PreviewSchema.round6(arrival.errorRadians()),
                PreviewSchema.round6(Math.toDegrees(arrival.errorRadians())),
                arrival.withinTolerance()));
        if (!arrival.withinTolerance()) {
          String goalRef = "goal-" + arrival.goalIndex();
          Integer line =
              arrival.goalIndex() >= 0
                      && arrival.goalIndex() < wpLines.size()
                      && wpLines.get(arrival.goalIndex()) > 0
                  ? wpLines.get(arrival.goalIndex())
                  : segDeclLine;
          diagnostics.add(
              new DiagnosticEntry(
                  "WARNING",
                  "HEADING_TOLERANCE_MISS",
                  "HEADING_TOLERANCE_MISS " + goalRef,
                  segId,
                  goalRef,
                  null,
                  line));
        }
      }

      // Build WaypointEntry list for this segment
      for (int g = 0; g < goals.size(); g++) {
        Goal goal = goals.get(g);
        int line = g < wpLines.size() ? wpLines.get(g) : -1;
        var sourceSpan = SourceSpanAnalyzer.findWaypointSpan(waypointSpans, line);
        waypoints.add(
            buildWaypointEntry(
                globalWaypointIndex++,
                segId,
                segIdx,
                g,
                goal,
                builtPath,
                closestPass[g],
                sourceSpan));
      }

      cumulativeLength += builtPath.length();
      simMsAccum += (System.nanoTime() - segSimStart) / 1_000_000.0;
    }

    String dominantLimiter = "TOP_SPEED";
    int bestCount = 0;
    for (Map.Entry<LimitingFactor, Integer> e : limiterCounts.entrySet()) {
      if (e.getValue() > bestCount) {
        bestCount = e.getValue();
        dominantLimiter = e.getKey().name();
      }
    }

    KeepoutSet keepoutSet = buildKeepoutSet(clearance);
    double totalMs = request.compileMs() + (System.nanoTime() - startNs) / 1_000_000.0;

    return new PreviewResult(
        PreviewSchema.SCHEMA_VERSION,
        autoClass.getName(),
        effectiveScope,
        activeSegmentId,
        segmentDescriptors,
        profile.id(),
        allianceName,
        request.seed(),
        PreviewSchema.round6(simTime),
        new PhaseTimings(
            PreviewSchema.round6(request.compileMs()),
            PreviewSchema.round6(planMsAccum),
            PreviewSchema.round6(simMsAccum),
            PreviewSchema.round6(totalMs)),
        request.buildIdentity(),
        new RobotFootprint(
            PreviewSchema.round6(profile.wheelBaseMeters()),
            PreviewSchema.round6(profile.trackWidthMeters()),
            PreviewSchema.round6(profile.bumperHalfLengthMeters()),
            PreviewSchema.round6(profile.bumperHalfWidthMeters()),
            PreviewSchema.round6(profile.bumperHalfLengthMeters() * 2.0),
            PreviewSchema.round6(profile.bumperHalfWidthMeters() * 2.0),
            PreviewSchema.round6(profile.intakeExtensionMeters()),
            PreviewSchema.round6(
                profile.bumperHalfLengthMeters() + profile.intakeExtensionMeters())),
        new PoseEntry(
            PreviewSchema.round6(entryPose.getX()),
            PreviewSchema.round6(entryPose.getY()),
            PreviewSchema.round6(entryPose.getRotation().getRadians()),
            PreviewSchema.round6(entryPose.getRotation().getDegrees())),
        waypoints,
        new MathPathEntry(
            PreviewSchema.round6(cumulativeLength), mathPieces, mathAnchors, sampledMathPoses),
        keepoutSet,
        trajectory,
        passedMarkers,
        headingArrivals,
        dominantLimiter,
        PreviewSchema.round6(minSlowdownRatio),
        diagnostics);
  }

  private static Object allocateWithoutConstructor(Class<?> clazz) throws Exception {
    Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
    Field theUnsafe = unsafeClass.getDeclaredField("theUnsafe");
    theUnsafe.setAccessible(true);
    Object unsafe = theUnsafe.get(null);
    Method allocateInstance = unsafeClass.getMethod("allocateInstance", Class.class);
    return allocateInstance.invoke(unsafe, clazz);
  }

  private static KeepoutSet buildKeepoutSet(FieldClearance clearance) {
    List<List<Translation2d>> rawPolys = clearance.keepouts();
    double margin = clearance.margin();
    List<KeepoutPolygon> hardPolygons = new ArrayList<>(rawPolys.size());
    List<KeepoutPolygon> marginPolygons = new ArrayList<>(rawPolys.size());

    for (int k = 0; k < rawPolys.size(); k++) {
      List<Translation2d> poly = rawPolys.get(k);
      String id = k < KEEPOUT_NAMES.size() ? KEEPOUT_NAMES.get(k) : "keepout-" + k;
      List<PointEntry> hardPts = new ArrayList<>(poly.size());
      for (Translation2d v : poly) {
        hardPts.add(new PointEntry(PreviewSchema.round6(v.getX()), PreviewSchema.round6(v.getY())));
      }
      hardPolygons.add(new KeepoutPolygon(id, hardPts));

      // Expand convex CCW polygon outward by clearance margin
      int n = poly.size();
      double[] nx = new double[n];
      double[] ny = new double[n];
      for (int i = 0; i < n; i++) {
        Translation2d a = poly.get(i);
        Translation2d b = poly.get((i + 1) % n);
        double ex = b.getX() - a.getX();
        double ey = b.getY() - a.getY();
        double len = Math.max(1e-9, Math.hypot(ex, ey));
        nx[i] = ey / len;
        ny[i] = -ex / len;
      }
      List<PointEntry> marginPts = new ArrayList<>(n);
      for (int i = 0; i < n; i++) {
        int prev = (i + n - 1) % n;
        double bisX = nx[prev] + nx[i];
        double bisY = ny[prev] + ny[i];
        double dot = 1.0 + nx[prev] * nx[i] + ny[prev] * ny[i];
        double scale = dot > 1e-3 ? margin / dot : margin;
        Translation2d v = poly.get(i);
        marginPts.add(
            new PointEntry(
                PreviewSchema.round6(v.getX() + bisX * scale),
                PreviewSchema.round6(v.getY() + bisY * scale)));
      }
      marginPolygons.add(new KeepoutPolygon(id + "-margin", marginPts));
    }
    return new KeepoutSet(PreviewSchema.round6(margin), hardPolygons, marginPolygons);
  }

  private static WaypointEntry buildWaypointEntry(
      int globalIndex,
      String segmentId,
      int segmentIndex,
      int goalIndexInSeg,
      Goal goal,
      PathBuilder.Path builtPath,
      double closestPassMeters,
      @Nullable WaypointSpan sourceSpan) {
    double withinMeters =
        goal.passType() == Goal.PassType.STOP
            ? goal.finishTolerance().positionMeters()
            : goal.withinMeters();

    boolean hasHeadingTarget = false;
    Double targetHeadingRadians = null;
    Double targetHeadingDegrees = null;
    Double headingToleranceRadians = null;
    Double headingToleranceDegrees = null;
    boolean explicitHeadingTolerance = false;
    String gatePolicy = "NONE";
    String turnDirection = "NONE";
    double anchorOffsetMeters = 0.0;

    HeadingPlan hp = goal.heading();
    if (hp != null) {
      hasHeadingTarget = true;
      gatePolicy = hp.gate().name();
      turnDirection = hp.direction().name();
      anchorOffsetMeters = hp.offsetMeters();
      explicitHeadingTolerance = hp.explicitTolerance();

      Rotation2d targetRot = resolveHeadingTarget(hp, goal, builtPath, goalIndexInSeg);
      targetHeadingRadians = PreviewSchema.round6(targetRot.getRadians());
      targetHeadingDegrees = PreviewSchema.round6(targetRot.getDegrees());

      if (hp.explicitTolerance() || hp.gate() == HeadingPlan.GatePolicy.HARD) {
        headingToleranceRadians = PreviewSchema.round6(hp.toleranceRadians());
        headingToleranceDegrees = PreviewSchema.round6(Math.toDegrees(hp.toleranceRadians()));
      } else if (goal.passType() == Goal.PassType.STOP) {
        headingToleranceRadians = PreviewSchema.round6(goal.finishTolerance().headingRadians());
        headingToleranceDegrees =
            PreviewSchema.round6(Math.toDegrees(goal.finishTolerance().headingRadians()));
      }
    } else if (goal.passType() == Goal.PassType.STOP) {
      hasHeadingTarget = true;
      gatePolicy = "SOFT";
      turnDirection = "SHORTEST";
      Rotation2d stopRot = goal.pose().getRotation();
      targetHeadingRadians = PreviewSchema.round6(stopRot.getRadians());
      targetHeadingDegrees = PreviewSchema.round6(stopRot.getDegrees());
      headingToleranceRadians = PreviewSchema.round6(goal.finishTolerance().headingRadians());
      headingToleranceDegrees =
          PreviewSchema.round6(Math.toDegrees(goal.finishTolerance().headingRadians()));
    }

    return new WaypointEntry(
        globalIndex,
        segmentId,
        segmentIndex,
        "goal-" + goalIndexInSeg,
        goal.passType().name(),
        PreviewSchema.round6(goal.pose().getX()),
        PreviewSchema.round6(goal.pose().getY()),
        PreviewSchema.round6(withinMeters),
        hasHeadingTarget,
        targetHeadingRadians,
        targetHeadingDegrees,
        headingToleranceRadians,
        headingToleranceDegrees,
        explicitHeadingTolerance,
        gatePolicy,
        turnDirection,
        PreviewSchema.round6(anchorOffsetMeters),
        goal.marker() != null ? goal.marker().name() : null,
        PreviewSchema.round6(closestPassMeters),
        sourceSpan != null ? sourceSpan.startLine() : null,
        sourceSpan != null ? sourceSpan.endLine() : null);
  }

  private static ExtractedAuto extractAuto(Class<?> autoClass, RobotProfile profile) {
    List<DiagnosticEntry> diagnostics = new ArrayList<>();
    Map<String, AutoSegment> staticSegmentsByField = new LinkedHashMap<>();
    Pose2d discoveredStartPose = null;

    for (Field field : autoClass.getDeclaredFields()) {
      if (!Modifier.isStatic(field.getModifiers())) {
        continue;
      }
      field.setAccessible(true);
      try {
        if (AutoSegment.class.isAssignableFrom(field.getType())) {
          Object val = field.get(null);
          if (val instanceof AutoSegment seg) {
            staticSegmentsByField.put(field.getName(), seg);
          }
        } else if (Pose2d.class.isAssignableFrom(field.getType()) && discoveredStartPose == null) {
          String name = field.getName().toUpperCase(Locale.ROOT);
          if (name.contains("START")) {
            Object val = field.get(null);
            if (val instanceof Pose2d pose) {
              discoveredStartPose = pose;
            }
          }
        }
      } catch (ReflectiveOperationException ignored) {
        // continue
      }
    }

    List<NamedSegment> orderedSegments = new ArrayList<>();
    // Try instantiating TrailblazerAuto(AutoContext) to capture super(...) startPose and segments
    try {
      ClassLoader cl = autoClass.getClassLoader();
      Class<?> autoContextClass = Class.forName("frc.robot.autos.AutoContext", true, cl);
      Class<?> robotManagerClass = Class.forName("frc.robot.robot_manager.RobotManager", true, cl);
      Class<?> tbAutoBaseClass = Class.forName("frc.robot.autos.TrailblazerAuto", true, cl);
      if (tbAutoBaseClass.isAssignableFrom(autoClass)) {
        Constructor<?> autoCtor = autoClass.getDeclaredConstructor(autoContextClass);
        autoCtor.setAccessible(true);
        Object dummyRobot = allocateWithoutConstructor(robotManagerClass);
        Trailblazer dummyTb =
            new Trailblazer(() -> 0.0, profile.spec(), profile.calibration(), DT_SECONDS);
        Constructor<?> ctxCtor =
            autoContextClass.getDeclaredConstructor(
                robotManagerClass, Trailblazer.class, Alliance.class);
        ctxCtor.setAccessible(true);
        Object ctx = ctxCtor.newInstance(dummyRobot, dummyTb, Alliance.Blue);
        Object autoInstance = autoCtor.newInstance(ctx);

        Method startPoseMethod = tbAutoBaseClass.getDeclaredMethod("startPose");
        startPoseMethod.setAccessible(true);
        Object optPose = startPoseMethod.invoke(autoInstance);
        if (optPose instanceof Optional<?> opt
            && opt.isPresent()
            && opt.get() instanceof Pose2d p) {
          discoveredStartPose = p;
        }

        Field previewSegsField = tbAutoBaseClass.getDeclaredField("alliancePreviewSegments");
        previewSegsField.setAccessible(true);
        Object segListObj = previewSegsField.get(autoInstance);
        if (segListObj instanceof List<?> segList) {
          Set<String> matchedFields = new LinkedHashSet<>();
          for (Object item : segList) {
            if (item instanceof AutoSegment seg) {
              String matchedFieldName = null;
              for (Map.Entry<String, AutoSegment> entry : staticSegmentsByField.entrySet()) {
                if (!matchedFields.contains(entry.getKey())
                    && (Objects.equals(entry.getValue(), seg)
                        || entry.getValue().id().equals(seg.id()))) {
                  matchedFieldName = entry.getKey();
                  matchedFields.add(matchedFieldName);
                  break;
                }
              }
              if (matchedFieldName == null) {
                matchedFieldName = seg.id();
              }
              var authored = staticSegmentsByField.getOrDefault(matchedFieldName, seg);
              orderedSegments.add(new NamedSegment(matchedFieldName, seg, authored));
            }
          }
        }
      }
    } catch (Throwable ex) {
      // Fallback to static fields or methods below.
    }

    // Add any static AutoSegment fields that weren't already in orderedSegments
    for (Map.Entry<String, AutoSegment> entry : staticSegmentsByField.entrySet()) {
      boolean alreadyPresent =
          orderedSegments.stream()
              .anyMatch(ns ->
                  ns.fieldName().equals(entry.getKey())
                      || ns.segment().id().equals(entry.getValue().id()));
      if (!alreadyPresent) {
        orderedSegments.add(new NamedSegment(entry.getKey(), entry.getValue()));
      }
    }

    // If no segments found yet, check zero-arg static methods returning AutoSegment
    if (orderedSegments.isEmpty()) {
      for (Method method : autoClass.getDeclaredMethods()) {
        if (Modifier.isStatic(method.getModifiers())
            && method.getParameterCount() == 0
            && AutoSegment.class.isAssignableFrom(method.getReturnType())) {
          method.setAccessible(true);
          try {
            Object val = method.invoke(null);
            if (val instanceof AutoSegment seg) {
              orderedSegments.add(new NamedSegment(method.getName(), seg));
            }
          } catch (ReflectiveOperationException ignored) {
            // ignore
          }
        }
      }
    }

    if (discoveredStartPose == null) {
      discoveredStartPose =
          new Pose2d(FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getTranslation(), Rotation2d.k180deg);
    }

    if (orderedSegments.isEmpty()) {
      diagnostics.add(
          new DiagnosticEntry(
              "WARNING",
              "NO_SEGMENTS_DEFINED",
              "NO_SEGMENTS_DEFINED " + autoClass.getSimpleName(),
              null,
              null,
              null,
              null));
    }

    return new ExtractedAuto(discoveredStartPose, orderedSegments, diagnostics);
  }

  private static List<Translation2d> flipPolygon(List<Translation2d> bluePolygon) {
    List<Translation2d> out = new ArrayList<>(bluePolygon.size());
    for (Translation2d pt : bluePolygon) {
      out.add(FIELD_2026.flipPoint(pt));
    }
    return List.copyOf(out);
  }

  private static List<Translation2d> rectangle(
      double left, double bottom, double right, double top) {
    return ImmutableList.of(
        new Translation2d(left, bottom),
        new Translation2d(right, bottom),
        new Translation2d(right, top),
        new Translation2d(left, top));
  }

  private static Rotation2d resolveHeadingTarget(
      HeadingPlan hp, Goal goal, PathBuilder.Path builtPath, int goalIndexInSeg) {
    if (hp.target() instanceof HeadingPlan.Target.Fixed fixed) {
      return fixed.angle();
    }
    if (hp.target() instanceof HeadingPlan.Target.Point aim) {
      Translation2d pos = goal.pose().getTranslation();
      Translation2d delta = aim.target().minus(pos);
      if (delta.getNorm() > 1e-6) {
        return Rotation2d.fromRadians(
            MathUtil.angleModulus(
                Math.atan2(delta.getY(), delta.getX()) - aim.mechanismOffset().getRadians()));
      }
      return goal.pose().getRotation();
    }
    if (hp.target() instanceof HeadingPlan.Target.Travel) {
      double anchor =
          goalIndexInSeg < builtPath.anchors().size()
              ? builtPath.anchors().get(goalIndexInSeg)
              : builtPath.length();
      var sample = builtPath.sample(anchor);
      if (sample.tangent().getNorm() > 1e-6) {
        return sample.tangent().getAngle();
      }
    }
    return goal.pose().getRotation();
  }

  private static Pose2d terminalGoalPose(Goal terminal, Pose2d fallbackStart) {
    if (terminal.heading() != null) {
      if (terminal.heading().target() instanceof HeadingPlan.Target.Fixed fixed) {
        return new Pose2d(terminal.pose().getTranslation(), fixed.angle());
      }
      if (terminal.heading().target() instanceof HeadingPlan.Target.Point aim) {
        Translation2d delta = aim.target().minus(terminal.pose().getTranslation());
        if (delta.getNorm() > 1e-6) {
          Rotation2d rot =
              Rotation2d.fromRadians(
                  MathUtil.angleModulus(
                      Math.atan2(delta.getY(), delta.getX()) - aim.mechanismOffset().getRadians()));
          return new Pose2d(terminal.pose().getTranslation(), rot);
        }
      }
    }
    return terminal.pose();
  }

  private PreviewRunner() {}
}
