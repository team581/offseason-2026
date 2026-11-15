package com.team581.trailblazer3.runtime;

import com.team581.trailblazer3.math.DrivetrainLimits;
import com.team581.trailblazer3.math.HeadingPlanner;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.math.RotationBudgetEstimator;
import com.team581.trailblazer3.math.VectorLimiter;
import com.team581.trailblazer3.math.VelocityProfiler;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TargetLossPolicy;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.ChassisAccelerations;
import com.team581.trailblazer3.model.TickResult.GoalPassageStatus;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.model.TimestampedTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleUnaryOperator;

public final class Trailblazer {
  public record MotionTelemetry(
      double maxSpeed,
      double availableAcceleration,
      double maxBraking,
      double maxAngularSpeed,
      double maxAngularAcceleration,
      double profileSpeed,
      double profileAcceleration,
      double commandedAcceleration,
      double distanceRemaining,
      double positionError,
      double dwellSeconds) {}

  public enum Ownership {
    NONE,
    ROTATION,
    TRANSLATION_AND_ROTATION
  }

  private record Barrier(double progress, boolean closed) {}

  private static final double CORRIDOR = 0.30;

  private static final double RELATCH_DWELL = 0.15;

  private static final double MAX_TICK_INTERVAL = 0.10;

  private static final double MAX_SNAPSHOT_AGE_SECONDS = 0.10;

  private static final double MAX_SNAPSHOT_FUTURE_SKEW_SECONDS = 0.025;

  private static final double SNAPSHOT_GRACE_SECONDS = 0.50;

  /** Samples discrete poses along a path geometry with planned per-goal headings for previews. */
  public static Pose2d[] sampleSegmentPoses(
      Pose2d startPose, PathBuilder.Path geometry, List<Goal> goals, double stepMeters) {
    if (geometry == null || goals == null || goals.isEmpty() || !(stepMeters > 0)) {
      return new Pose2d[0];
    }
    return sampleSegmentPoses(startPose, geometry, goals, null, 0.0, stepMeters, new double[5]);
  }

  public static AutoSegment.Builder segment() {
    return new AutoSegment.Builder("segment");
  }

  public static AutoSegment.Builder segment(String id) {
    return new AutoSegment.Builder(id);
  }

  private static String corruptSnapshotFault(RobotStateSnapshot snapshot, double now) {
    double timestamp = snapshot.poseTimestampSeconds();
    if (!Double.isFinite(timestamp)) {
      return "Odometry timestamp is nonfinite";
    }
    if (now - timestamp < -MAX_SNAPSHOT_FUTURE_SKEW_SECONDS) {
      return "Odometry timestamp is ahead of the planner clock";
    }
    var pose = snapshot.measuredPose();
    if (!Double.isFinite(pose.getTranslation().getNorm())
        || !Double.isFinite(pose.getRotation().getRadians())) {
      return "Measured pose is nonfinite";
    }
    if (!finite(snapshot.measuredFieldSpeeds())
        || snapshot.appliedFieldSpeeds().filter(speed -> !finite(speed)).isPresent()) {
      return "Measured or applied chassis velocity is nonfinite";
    }
    if (!validPower(snapshot.swerveSupplyCurrentLimitAmps().orElse(1))
        || !validPower(snapshot.restingBatteryVoltageVolts().orElse(12))) {
      return "Invalid drivetrain power limits";
    }
    return null;
  }

  private static double distanceToSegment(
      Translation2d point, Translation2d start, Translation2d end) {
    var delta = end.minus(start);
    double squared = delta.dot(delta);
    double fraction =
        squared > 1e-12 ? MathUtil.clamp(point.minus(start).dot(delta) / squared, 0, 1) : 0;
    return point.getDistance(start.plus(delta.times(fraction)));
  }

  private static boolean evaluateFixedSpan(
      PathBuilder.Path geometry,
      FieldClearance clearance,
      double[] sampleScratch,
      double[] spanState,
      boolean escapeNonDecreasing,
      boolean checkEscape,
      double escapeWindow,
      double required,
      double safeTarget,
      double segStart,
      double anchor,
      double spanEnd,
      double holdUntil,
      double currentHeading,
      double startCos,
      double startSin,
      double diff,
      double spanEndHeading,
      int headingIntervals,
      double headingAllowance) {
    double minPath = spanState[0];
    double minBeyondEscape = spanState[1];
    double prevNominal = spanState[2];
    double escapeEndClearance = spanState[3];
    for (double progress = segStart; ; ) {
      geometry.sampleInto(progress, sampleScratch);
      double px = sampleScratch[0];
      double py = sampleScratch[1];
      boolean holdingNow = progress < holdUntil - 1e-9;
      double pointClearance;
      if (holdingNow) {
        pointClearance = clearance.clearance(px, py, startCos, startSin);
      } else {
        double angleBound = clearance.minimumAngleClearance(px, py);
        if ((!checkEscape || progress > escapeWindow + 1e-9) && angleBound > safeTarget + 0.02) {
          minPath = Math.min(minPath, angleBound);
          if (progress > escapeWindow + 1e-9) {
            minBeyondEscape = Math.min(minBeyondEscape, angleBound);
          }
          if (progress >= spanEnd - 1e-9) {
            break;
          }
          double remaining = spanEnd - progress;
          double step = Math.min(0.25, Math.min((angleBound - safeTarget) / 2.0, remaining));
          minPath = Math.min(minPath, angleBound - step);
          if (progress > escapeWindow + 1e-9) {
            minBeyondEscape = Math.min(minBeyondEscape, angleBound - step);
          }
          progress = Math.min(spanEnd, progress + Math.max(step, 1e-6));
          continue;
        }
        pointClearance =
            angleBound >= safeTarget && angleBound > 0.0
                ? angleBound
                : Math.max(
                    angleBound,
                    sweptClearanceAt(
                        clearance,
                        px,
                        py,
                        currentHeading,
                        diff,
                        headingIntervals,
                        headingAllowance));
      }
      minPath = Math.min(minPath, pointClearance);
      if (checkEscape && progress <= escapeWindow + 1e-9) {
        double turnStart = holdUntil > segStart ? Math.min(holdUntil, anchor) : segStart;
        double nominalAngle;
        if (holdingNow) {
          nominalAngle = currentHeading;
        } else {
          nominalAngle =
              anchor > turnStart + 1e-9
                  ? currentHeading
                      + MathUtil.clamp((progress - turnStart) / (anchor - turnStart), 0, 1) * diff
                  : spanEndHeading;
        }
        double nominal =
            clearance.clearance(px, py, Math.cos(nominalAngle), Math.sin(nominalAngle));
        if (nominal < prevNominal - 1e-6) {
          escapeNonDecreasing = false;
        }
        prevNominal = nominal;
        escapeEndClearance = nominal;
      } else {
        minBeyondEscape = Math.min(minBeyondEscape, pointClearance);
      }
      if (progress >= spanEnd - 1e-9) {
        break;
      }
      if (pointClearance <= required && !checkEscape && progress > escapeWindow + 1e-9) {
        break;
      }
      double remaining = spanEnd - progress;
      double step;
      if (holdingNow) {
        double holdRemaining = holdUntil - progress;
        if (checkEscape && progress < escapeWindow - 1e-9) {
          holdRemaining = Math.min(holdRemaining, escapeWindow - progress);
        }
        step = Math.min(0.025, Math.min(holdRemaining, remaining));
      } else if (checkEscape && progress < escapeWindow - 1e-9) {
        step = Math.min(0.025, Math.min(escapeWindow - progress, remaining));
      } else if (pointClearance > required + 0.02) {
        // Translation by ds along arc length can reduce clearance by at most ds.
        step = Math.min(0.25, Math.min((pointClearance - required) / 2.0, remaining));
        minPath = Math.min(minPath, pointClearance - step);
        minBeyondEscape = Math.min(minBeyondEscape, pointClearance - step);
      } else {
        step = Math.min(0.025, remaining);
      }
      progress = Math.min(spanEnd, progress + Math.max(step, 1e-6));
    }
    spanState[0] = minPath;
    spanState[1] = minBeyondEscape;
    spanState[2] = prevNominal;
    spanState[3] = escapeEndClearance;
    return escapeNonDecreasing;
  }

  private static double findEarliestSafeTurnStart(
      PathBuilder.Path geometry,
      FieldClearance clearance,
      double[] sampleScratch,
      double anchor,
      double spanEnd,
      double currentHeading,
      double diff,
      int headingIntervals,
      double headingAllowance,
      double target) {
    if (spanEnd > anchor + 1e-9) {
      int tailSteps = Math.max(1, (int) Math.ceil((spanEnd - anchor) / 0.025));
      for (int i = tailSteps; i >= 1; i--) {
        double s = anchor + ((spanEnd - anchor) * i) / tailSteps;
        double clearanceAtS =
            sweptOrBoundClearance(
                geometry,
                clearance,
                sampleScratch,
                s,
                currentHeading,
                diff,
                headingIntervals,
                headingAllowance,
                target);
        if (clearanceAtS < target || clearanceAtS <= 0.0) {
          return -1.0;
        }
      }
    }
    int steps = Math.max(1, (int) Math.ceil(anchor / 0.025));
    double earliestSafe = -1.0;
    for (int i = steps; i >= 1; i--) {
      double s = i == steps ? anchor : Math.min(anchor, i * 0.025);
      double clearanceAtS =
          sweptOrBoundClearance(
              geometry,
              clearance,
              sampleScratch,
              s,
              currentHeading,
              diff,
              headingIntervals,
              headingAllowance,
              target);
      if (clearanceAtS < target || clearanceAtS <= 0.0) {
        break;
      }
      earliestSafe = s;
    }
    return earliestSafe;
  }

  private static boolean finite(ChassisSpeeds speed) {
    return Double.isFinite(speed.vxMetersPerSecond)
        && Double.isFinite(speed.vyMetersPerSecond)
        && Double.isFinite(speed.omegaRadiansPerSecond);
  }

  private static double plannedHeadingAtRadians(
      Pose2d startPose,
      PathBuilder.Path geometry,
      List<Goal> goals,
      HeadingPlan inheritedHeading,
      double firstTurnStartProgress,
      double s,
      double[] sampleScratch) {
    double segStart = 0;
    double current = startPose.getRotation().getRadians();
    boolean appliedDelay = false;
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      var keyframe = goal.heading();
      if (keyframe == null && i == 0 && inheritedHeading != null) {
        keyframe = inheritedHeading;
      }
      if (keyframe == null && goal.passType() == Goal.PassType.STOP) {
        keyframe = HeadingPlan.to(goal.pose().getRotation());
      }
      if (keyframe == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(
              geometry.anchors().get(i) + keyframe.offsetMeters(), segStart, geometry.length());
      double turnStart = segStart;
      if (!appliedDelay) {
        appliedDelay = true;
        if (firstTurnStartProgress > segStart) {
          turnStart = Math.min(firstTurnStartProgress, anchor);
        }
      }
      double target = current;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        target = fixed.angle().getRadians();
      } else if (keyframe.target() instanceof HeadingPlan.Target.Travel) {
        geometry.sampleInto(s, sampleScratch);
        if (Math.hypot(sampleScratch[2], sampleScratch[3]) > 1e-6) {
          target = Math.atan2(sampleScratch[3], sampleScratch[2]);
        }
      } else if (keyframe.target() instanceof HeadingPlan.Target.Point fixed) {
        geometry.sampleInto(s, sampleScratch);
        double dx = fixed.target().getX() - sampleScratch[0];
        double dy = fixed.target().getY() - sampleScratch[1];
        if (Math.hypot(dx, dy) >= 0.25) {
          target = Math.atan2(dy, dx) - fixed.mechanismOffset().getRadians();
        }
      }
      if (s <= anchor || i == goals.size() - 1) {
        double fraction =
            anchor > turnStart + 1e-9
                ? MathUtil.clamp((s - turnStart) / (anchor - turnStart), 0, 1)
                : (s >= turnStart - 1e-9 ? 1.0 : 0.0);
        double diff = MathUtil.angleModulus(target - current);
        if (keyframe.direction() == HeadingPlan.TurnDirection.CW && diff > 0) {
          diff -= 2 * Math.PI;
        } else if (keyframe.direction() == HeadingPlan.TurnDirection.CCW && diff < 0) {
          diff += 2 * Math.PI;
        }
        return current + fraction * diff;
      }
      current = target;
      segStart = anchor;
    }
    return current;
  }

  private static Pose2d[] sampleSegmentPoses(
      Pose2d startPose,
      PathBuilder.Path geometry,
      List<Goal> goals,
      HeadingPlan initialInheritedHeading,
      double firstTurnStartProgress,
      double stepMeters,
      double[] sampleScratch) {
    double pathLength = geometry.length();
    int intervals = Math.max(1, (int) Math.ceil(pathLength / stepMeters));
    Pose2d[] poses = new Pose2d[intervals + 1];
    for (int i = 0; i <= intervals; i++) {
      double s = (pathLength * i) / intervals;
      geometry.sampleInto(s, sampleScratch);
      double px = sampleScratch[0];
      double py = sampleScratch[1];
      double headingRad =
          plannedHeadingAtRadians(
              startPose,
              geometry,
              goals,
              initialInheritedHeading,
              firstTurnStartProgress,
              s,
              sampleScratch);
      poses[i] = new Pose2d(px, py, Rotation2d.fromRadians(headingRad));
    }
    return poses;
  }

  private static double sweptClearanceAt(
      FieldClearance clearance,
      double px,
      double py,
      double currentHeading,
      double diff,
      int headingIntervals,
      double headingAllowance) {
    double pointClearance = Double.POSITIVE_INFINITY;
    for (int angleStep = 0; angleStep <= headingIntervals; angleStep++) {
      double angle =
          headingIntervals == 0
              ? currentHeading
              : currentHeading + (diff * angleStep) / headingIntervals;
      pointClearance =
          Math.min(pointClearance, clearance.clearance(px, py, Math.cos(angle), Math.sin(angle)));
    }
    return pointClearance - headingAllowance;
  }

  private static double sweptOrBoundClearance(
      PathBuilder.Path geometry,
      FieldClearance clearance,
      double[] sampleScratch,
      double s,
      double currentHeading,
      double diff,
      int headingIntervals,
      double headingAllowance,
      double target) {
    geometry.sampleInto(s, sampleScratch);
    double px = sampleScratch[0];
    double py = sampleScratch[1];
    double fastBound = clearance.minimumAngleClearance(px, py);
    if (fastBound >= target && fastBound > 0.0) {
      return fastBound;
    }
    return Math.max(
        fastBound,
        sweptClearanceAt(
            clearance, px, py, currentHeading, diff, headingIntervals, headingAllowance));
  }

  private static Translation2d translation(ChassisSpeeds speed) {
    return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
  }

  private static String unreadyOrStaleFault(RobotStateSnapshot snapshot, double now) {
    if (!snapshot.localizationValid()) {
      return "Localization reported an invalid pose";
    }
    double timestamp = snapshot.poseTimestampSeconds();
    if (timestamp < 0
        || (timestamp == 0 && now > MAX_SNAPSHOT_AGE_SECONDS)
        || now - timestamp > MAX_SNAPSHOT_AGE_SECONDS) {
      return "Odometry sample is older than 100 ms";
    }
    return null;
  }

  private static boolean validPower(double value) {
    return Double.isFinite(value) && value > 0;
  }

  private final TimeSource clock;
  private final RobotDrivetrainSpec robot;
  private final EmpiricalDriveCalibration calibration;
  private final double nominalDt;
  private final double squaredModuleRadii;
  private final Optional<FieldClearance> defaultFieldClearance;
  private Optional<FieldClearance> fieldClearance;
  private boolean marginWarning;
  private final Map<String, GoalPassageStatus> statuses = new LinkedHashMap<>();
  private Map<String, GoalPassageStatus> immutableStatuses = Map.of();
  private final Set<Enum<?>> passed = new LinkedHashSet<>();
  private final Map<String, TimestampedTarget<Pose2d>> lastTargets = new HashMap<>();
  private final Map<String, Pose2d> geometryTargets = new HashMap<>();
  private final Set<String> openedGates = new LinkedHashSet<>();
  private final VelocityProfiler.Workspace profilerWorkspace = new VelocityProfiler.Workspace();
  private final double[] clearanceSampleScratch = new double[5];
  private final double[] headingSampleScratch = new double[5];
  private final double[] spanStateScratch = new double[4];
  private AutoSegment plan;
  private boolean hasDynamicGoals;
  private List<Goal> resolvedGoals;
  private String headingId;
  private double headingDwell;
  private double targetDerivative;
  private double referenceAlpha;
  private boolean faceTravel;
  private boolean singularAim;
  private double gateWait;
  private Pose2d assistPose;
  private Rotation2d snapHeading;
  private boolean assist;
  private boolean pendingAssist;
  private MotionLimits assistProfile;
  private List<Goal> horizon;
  private PathBuilder.Path path;
  private int horizonStart;
  private int goalIndex;
  private long generation;
  private double highWater;
  private double closest;

  private double dwell;

  private double offTrack;
  private double divergence;
  private double activationTime;

  private double lastTime;

  private double targetHeading;

  private double horizonTurnStartProgress;

  private double horizonHoldHeadingRadians;

  private boolean terminalLatched;

  private double unhealthySnapshotSeconds;

  private HeadingPlan inheritedHeading;

  private boolean stopping;

  private RuntimeStatus stopStatus;

  private String stopDiagnostic;

  private Pose2d rebuildStartPose = Pose2d.kZero;
  private Translation2d lastPosition;

  private HeadingPlanner.State heading;

  private ChassisSpeeds previous;

  private DrivetrainLimits nominalLimits;

  private DrivetrainLimits limits;

  private TickResult result = zero(RuntimeStatus.CANCELLED, "");

  private MotionTelemetry telemetry;

  public Trailblazer(
      TimeSource clock,
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt) {
    this(clock, robot, calibration, nominalDt, Optional.empty());
  }

  public Trailblazer(
      TimeSource clock,
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      Optional<FieldClearance> fieldClearance) {
    this.defaultFieldClearance = fieldClearance;
    this.fieldClearance = fieldClearance;
    if (!Double.isFinite(nominalDt) || nominalDt < 0.004 || nominalDt > 0.020) {
      throw new IllegalArgumentException("Nominal period must be within 4–20 ms");
    }
    this.clock = clock;
    this.robot = robot;
    this.calibration = calibration;
    this.nominalDt = nominalDt;
    double radiiSum = 0;
    for (var module : robot.modulePositionsMeters()) {
      radiiSum += module.dot(module);
    }
    this.squaredModuleRadii = radiiSum;
  }

  public void activate(AutoSegment motionPlan) {
    activate(motionPlan, motionPlan.goals().get(0).id());
  }

  public void activate(AutoSegment motionPlan, FieldClearance clearance) {
    activate(motionPlan, motionPlan.goals().get(0).id(), Optional.of(clearance));
  }

  public void activate(AutoSegment motionPlan, String startAtGoalId) {
    activate(motionPlan, startAtGoalId, defaultFieldClearance);
  }

  public void activateWithoutClearance(AutoSegment motionPlan) {
    activateWithoutClearance(motionPlan, motionPlan.goals().get(0).id());
  }

  public void activateWithoutClearance(AutoSegment motionPlan, String startAtGoalId) {
    activate(motionPlan, startAtGoalId, Optional.empty());
  }

  public boolean atGoal() {
    return result.status() == RuntimeStatus.AT_GOAL;
  }

  public void cancel() {
    pendingAssist = false;
    unhealthySnapshotSeconds = 0;
    if (path != null && (result.status() == RuntimeStatus.RUNNING || atGoal())) {
      stop(RuntimeStatus.CANCELLED, "Cancelled");
    } else {
      result = zero(RuntimeStatus.CANCELLED, "Cancelled");
    }
  }

  public Optional<Pose2d> currentTargetPose() {
    if (path == null || heading == null) {
      return Optional.empty();
    }
    path.sampleInto(highWater, headingSampleScratch);
    return Optional.of(
        new Pose2d(
            headingSampleScratch[0],
            headingSampleScratch[1],
            Rotation2d.fromRadians(targetHeading)));
  }

  public void driveTo(Pose2d pose, MotionLimits profile) {
    assistPose = pose;
    snapHeading = null;
    assistProfile = profile;
    pendingAssist = true;
  }

  public boolean isFaulted() {
    return result.status() == RuntimeStatus.FAULT;
  }

  public TickResult lastResult() {
    return result;
  }

  public Optional<MotionTelemetry> motionTelemetry() {
    return Optional.ofNullable(telemetry);
  }

  public double nowSeconds() {
    return clock.getTimestampSeconds();
  }

  public Ownership ownership() {
    if (pendingAssist) {
      return snapHeading == null ? Ownership.TRANSLATION_AND_ROTATION : Ownership.ROTATION;
    }
    if (!assist || result.status() != RuntimeStatus.RUNNING || stopping) {
      return Ownership.NONE;
    }
    return snapHeading == null ? Ownership.TRANSLATION_AND_ROTATION : Ownership.ROTATION;
  }

  public boolean passedMarker(Enum<?> marker) {
    return passed.contains(marker);
  }

  public void preempt() {
    cancel();
  }

  public boolean preempt(boolean translationIntervention, boolean rotationIntervention) {
    var owner = ownership();
    if (owner != Ownership.NONE
        && (rotationIntervention
            || (owner == Ownership.TRANSLATION_AND_ROTATION && translationIntervention))) {
      cancel();
      return true;
    }
    return false;
  }

  public boolean release() {
    if (!atGoal() || horizonStart + horizon.size() >= plan.goals().size()) {
      return false;
    }
    horizonStart += horizon.size();
    startHorizon();
    return true;
  }

  public Pose2d[] sampledPathPoses(double stepMeters) {
    if (path == null || horizon == null || !(stepMeters > 0)) {
      return new Pose2d[0];
    }
    return sampleSegmentPoses(
        rebuildStartPose,
        path,
        horizon,
        inheritedHeading,
        horizonTurnStartProgress,
        stepMeters,
        headingSampleScratch);
  }

  public void snapAngle(Rotation2d angle, MotionLimits profile) {
    snapHeading = angle;
    assistPose = null;
    assistProfile = profile;
    pendingAssist = true;
  }

  public TickResult update(RobotStateSnapshot snapshot) {
    return update(
        new TickInput(clock.getTimestampSeconds(), snapshot, Map.of(), Map.of(), Optional.empty()));
  }

  public TickResult update(TickInput input) {
    double now = input.timestampSeconds();
    if (!Double.isFinite(now)) {
      result = zero(RuntimeStatus.FAULT, "Invalid tick interval");
      return result;
    }
    var snapshot = input.robotState();
    if (!snapshot.enabled()) {
      pendingAssist = false;
      unhealthySnapshotSeconds = 0;
      result = zero(RuntimeStatus.DISABLED, "");
      return result;
    }
    if (pendingAssist) {
      var angle = snapHeading;
      var target =
          angle == null ? assistPose : new Pose2d(snapshot.measuredPose().getTranslation(), angle);
      var goal = Goal.stop(target);
      if (angle == null) {
        goal = goal.heading(HeadingPlan.to(target.getRotation()).gate(HeadingPlan.GatePolicy.HARD));
      }
      activate(segment("assist").withLimits(assistProfile).addGoal(goal).build());
      assist = true;
      snapHeading = angle;
    }
    if (result.status() != RuntimeStatus.RUNNING
        && !(atGoal()
            && (stopping
                || (path != null
                    && horizon.get(horizon.size() - 1).passType()
                        == Goal.PassType.CRUISE_THROUGH)))) {
      return result;
    }
    boolean firstTick = Double.isNaN(lastTime);
    if (!firstTick && now - lastTime < -1e-4) {
      result = zero(RuntimeStatus.FAULT, "Invalid tick interval");
      return result;
    }
    if (!firstTick && now - lastTime < 1e-4) {
      return result;
    }
    double elapsed = firstTick ? nominalDt : now - lastTime;
    if (!Double.isFinite(elapsed) || elapsed <= 0) {
      result = zero(RuntimeStatus.FAULT, "Invalid tick interval");
      return result;
    }
    boolean missedTick = elapsed > MAX_TICK_INTERVAL;
    double dt = missedTick ? nominalDt : elapsed;
    lastTime = now;
    var pose = snapshot.measuredPose();
    var measuredSpeeds = snapshot.measuredFieldSpeeds();
    String corruptFault = corruptSnapshotFault(snapshot, now);
    if (corruptFault != null) {
      result = zero(RuntimeStatus.FAULT, corruptFault);
      return result;
    }
    String staleFault = unreadyOrStaleFault(snapshot, now);
    if (staleFault != null) {
      if (path == null) {
        unhealthySnapshotSeconds += dt;
        if (unhealthySnapshotSeconds <= SNAPSHOT_GRACE_SECONDS) {
          result = holdDuringSnapshotGrace(dt);
          return result;
        }
      }
      result = zero(RuntimeStatus.FAULT, staleFault);
      return result;
    }
    unhealthySnapshotSeconds = 0;
    if (Double.isNaN(activationTime)) {
      activationTime = now;
    }
    var measured = translation(measuredSpeeds);
    try {
      if (path == null || missedTick) {
        previous = snapshot.appliedFieldSpeeds().orElse(measuredSpeeds);
        double measuredHeading = pose.getRotation().getRadians();
        if (path != null) {
          measuredHeading =
              heading.angle() + MathUtil.angleModulus(measuredHeading - heading.angle());
        }
        heading = new HeadingPlanner.State(measuredHeading, previous.omegaRadiansPerSecond);
        referenceAlpha = 0;
      }
      if (path == null) {
        headingId = null;
        targetHeading = heading.angle();
      }
      if (missedTick) {
        // The loop did not observe the robot during the gap, so restart dwell checks.
        dwell = 0;
        headingDwell = 0;
        offTrack = 0;
        divergence = 0;
      }
      if (stopping) {
        return brake(pose, snapshot, dt);
      }
      if (!resolveGoals(
          input,
          path == null ? horizonStart : horizonStart + Math.min(goalIndex, horizon.size() - 1))) {
        stop(RuntimeStatus.FAULT, "Dynamic goal target lost");
        return brake(pose, snapshot, dt);
      }
      if (path == null) {
        rebuild(pose, horizonStart);
      } else if (goalIndex < horizon.size() && targetsMoved()) {
        rebuild(pose, horizonStart + goalIndex);
      }
      String diagnostic =
          missedTick ? "MISSED_TICK_RECOVERY" : (marginWarning ? "KEEPOUT_MARGIN_WARNING" : "");
      if (snapshot.localizationResetOccurred() && goalIndex < horizon.size()) {
        rebuild(pose, horizonStart + goalIndex);
        diagnostic = "RELATCH";
      }
      if (now - activationTime > (assist ? 3.0 : 14.5) && !atGoal()) {
        stop(RuntimeStatus.FAULT, "Timed out");
      }
      if (stopping) {
        return brake(pose, snapshot, dt);
      }
      double firstAllowed = goalIndex > 1 ? path.anchors().get(goalIndex - 2) : 0;
      double lastAllowed =
          goalIndex + 1 < horizon.size() ? path.anchors().get(goalIndex + 1) : path.length();
      var projected =
          path.project(
              pose.getTranslation(),
              Math.max(firstAllowed, highWater - 0.60),
              Math.min(lastAllowed, highWater + 1.50));
      if (snapHeading != null) {
        offTrack = 0;
      } else {
        offTrack =
            projected.distance() > CORRIDOR || highWater - projected.progress() > 0.55
                ? offTrack + dt
                : 0;
      }
      divergence = measured.getDistance(translation(previous)) > 1.0 ? divergence + dt : 0;
      if (divergence >= RELATCH_DWELL) {
        previous = measuredSpeeds;
        divergence = 0;
      }
      if (offTrack >= RELATCH_DWELL && !atGoal()) {
        rebuild(pose, horizonStart + goalIndex);
        projected = path.project(pose.getTranslation(), 0, 0);
        diagnostic = "RELATCH";
      }
      var barrier = headingBarrier(input, pose, measuredSpeeds, projected, dt);
      if (stopping) {
        return brake(pose, snapshot, dt);
      }
      if (Math.abs(projected.crossTrack()) <= CORRIDOR && projected.distance() <= CORRIDOR) {
        highWater = Math.max(highWater, Math.min(projected.progress(), barrier.progress()));
      }
      Set<Enum<?>> newMarkers = null;
      RuntimeStatus status = atGoal() ? RuntimeStatus.AT_GOAL : RuntimeStatus.RUNNING;
      while (goalIndex < horizon.size()) {
        var goal = horizon.get(goalIndex);
        if (goal.passType() == Goal.PassType.STOP) {
          break;
        }
        closest =
            Math.min(
                closest,
                distanceToSegment(
                    goal.pose().getTranslation(), lastPosition, pose.getTranslation()));
        if (highWater + 1e-9 < path.anchors().get(goalIndex)
            || (barrier.closed() && path.anchors().get(goalIndex) >= barrier.progress() - 1e-9)) {
          break;
        }
        if (closest > Math.max(CORRIDOR, 2 * goal.withinMeters() + 0.15)) {
          rebuild(pose, horizonStart + goalIndex);
          projected = path.project(pose.getTranslation(), 0, 0);
          diagnostic = "RELATCH";
          barrier = headingBarrier(input, pose, measuredSpeeds, projected, 0);
          break;
        }
        newMarkers = recordPass(goal, closest <= goal.withinMeters(), newMarkers);
        goalIndex++;
        closest = Double.POSITIVE_INFINITY;
        if (goal.passType() == Goal.PassType.CRUISE_THROUGH) {
          status = RuntimeStatus.AT_GOAL;
        }
      }
      lastPosition = pose.getTranslation();
      var terminal = resolvedGoals.get(horizonStart + horizon.size() - 1);
      double along = translation(previous).dot(projected.sample().tangent());
      DoubleUnaryOperator motor =
          speed ->
              limits.acceleration(
                  speed,
                  snapshot.swerveSupplyCurrentLimitAmps(),
                  snapshot.restingBatteryVoltageVolts());
      double progress = projected.progress();
      var downstreamSpans = collectDownstreamHeadingSpans(progress, barrier.progress());
      var profile =
          VelocityProfiler.profile(
              path,
              horizon,
              plan.zones(),
              progress,
              along,
              barrier.progress(),
              (node, out) ->
                  RotationBudgetEstimator.estimateInto(
                      limits,
                      node,
                      progress,
                      faceTravel ? path.curvatureAt(node) : targetDerivative,
                      heading.velocity(),
                      referenceAlpha,
                      targetHeading - heading.angle(),
                      downstreamSpans,
                      out),
              limits,
              motor,
              dt,
              profilerWorkspace);
      var driver =
          snapHeading == null
              ? Translation2d.kZero
              : input.driverFieldVelocityMpS().orElse(Translation2d.kZero);
      if (!Double.isFinite(driver.getNorm())) {
        throw new IllegalArgumentException("Invalid driver field velocity");
      }
      if (driver.getNorm() > limits.maxSpeed()) {
        driver = driver.times(limits.maxSpeed() / driver.getNorm());
      }
      double headingSpeed = snapHeading == null ? profile.speed() : driver.getNorm();
      double headingBudgetSpeed;
      if (snapHeading != null) {
        headingBudgetSpeed = driver.getNorm();
      } else {
        double zoneCap = limits.maxSpeed();
        var zones = plan.zones();
        for (int i = 0; i < zones.size(); i++) {
          var zone = zones.get(i);
          if (progress >= zone.startMeters() && progress <= zone.endMeters()) {
            zoneCap = Math.min(zoneCap, zone.maxSpeed());
          }
        }
        double remainingToBarrier = Math.max(0.0, barrier.progress() - progress);
        double translationEnvelope =
            Math.min(
                zoneCap,
                Math.sqrt(
                    2.0 * Math.min(limits.braking(), limits.friction()) * remainingToBarrier));
        headingBudgetSpeed = Math.max(profile.speed(), translationEnvelope);
      }
      var turn =
          HeadingPlanner.step(
              heading,
              targetHeading,
              targetDerivative,
              pose.getRotation().getRadians(),
              previous.omegaRadiansPerSecond,
              headingSpeed,
              headingBudgetSpeed,
              limits,
              dt);
      heading = turn.reference();
      referenceAlpha = turn.acceleration();
      double traction = limits.availableTraction(heading.velocity(), referenceAlpha);
      double omega = turn.command();
      var tangent = path.sample(profile.tangentProgress());
      double tau = Math.max(0.12, 4 * (limits.lag() + nominalDt / 2));
      double crossTrackCap = Math.min(CORRIDOR / tau, 0.30 * limits.maxSpeed());
      var demand =
          tangent
              .tangent()
              .times(profile.speed())
              .plus(
                  projected
                      .sample()
                      .normal()
                      .times(
                          MathUtil.clamp(
                              -projected.crossTrack() / tau, -crossTrackCap, crossTrackCap)));
      var error = path.sample(barrier.progress()).position().minus(pose.getTranslation());
      double distance = error.getNorm();
      terminalLatched = distance <= (terminalLatched ? 0.12 : 0.08);
      double beta =
          terminalLatched || projected.progress() >= barrier.progress()
              ? 1
              : MathUtil.clamp(
                  (0.25 - Math.max(barrier.progress() - projected.progress(), distance)) / 0.17,
                  0,
                  1);
      double terminalSpeed =
          Math.min(
              limits.maxSpeed(),
              Math.min(
                  Math.sqrt(2 * Math.min(limits.braking(), traction) * distance),
                  distance / (0.85 * tau)));
      var terminalDemand =
          distance > 1e-9 ? error.times(terminalSpeed / distance) : Translation2d.kZero;
      demand = demand.times(1 - beta).plus(terminalDemand.times(beta));
      var oldVelocity = translation(previous);
      if (snapHeading != null) {
        demand = driver;
      } else if (profile.infeasible() && beta < 1) {
        double arrest = Math.min(Math.min(limits.braking(), traction) * dt, oldVelocity.getNorm());
        demand =
            oldVelocity.getNorm() > 1e-9
                ? oldVelocity.times(1 - arrest / oldVelocity.getNorm())
                : Translation2d.kZero;
      }
      double acceleration = motor.applyAsDouble(oldVelocity.getNorm());
      var limited =
          VectorLimiter.project(
              demand, oldVelocity, pose.getRotation(), omega, traction, acceleration, limits, dt);
      if (limited.infeasible()) {
        double maxDeltaOmega = limits.maxAlpha() * dt;
        double prevOmega = previous.omegaRadiansPerSecond;
        double[] omegaCandidates = {
          prevOmega,
          MathUtil.clamp(0.5 * prevOmega, prevOmega - maxDeltaOmega, prevOmega + maxDeltaOmega),
          MathUtil.clamp(0.0, prevOmega - maxDeltaOmega, prevOmega + maxDeltaOmega)
        };
        for (double candidateOmega : omegaCandidates) {
          if (Math.abs(candidateOmega - omega) <= 1e-9) {
            continue;
          }
          var retry =
              VectorLimiter.project(
                  demand,
                  oldVelocity,
                  pose.getRotation(),
                  candidateOmega,
                  traction,
                  acceleration,
                  limits,
                  dt);
          if (!retry.infeasible()) {
            omega = candidateOmega;
            limited = retry;
            break;
          }
        }
      }
      var velocity = limited.velocity();
      var tolerance = terminal.finishTolerance();
      if (terminal.passType() == Goal.PassType.STOP) {
        boolean translationReady =
            snapHeading != null
                || (pose.getTranslation().getDistance(terminal.pose().getTranslation())
                        <= tolerance.positionMeters()
                    && measured.getNorm() <= tolerance.linearSpeed()
                    && velocity.getNorm() <= tolerance.linearSpeed());
        boolean ready =
            !barrier.closed()
                && !singularAim
                && translationReady
                && Math.abs(targetHeading - heading.angle()) < Math.PI / 2
                && Math.abs(MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()))
                    <= tolerance.headingRadians()
                && Math.abs(measuredSpeeds.omegaRadiansPerSecond) <= tolerance.angularSpeed()
                && Math.abs(omega) <= tolerance.angularSpeed()
                && !limited.infeasible();
        dwell = ready ? dwell + dt : 0;
        if (ready
            && dwell + 1e-9 >= tolerance.dwellSeconds()
            && (snapHeading != null
                || oldVelocity.getNorm() <= Math.min(limits.braking(), traction) * dt + 1e-9)
            && Math.abs(previous.omegaRadiansPerSecond) <= limits.maxAlpha() * dt + 1e-9) {
          newMarkers = recordPass(terminal, true, newMarkers);
          status = RuntimeStatus.AT_GOAL;
          if (snapHeading == null) {
            velocity = Translation2d.kZero;
          }
          omega = 0;
        }
      }
      double ffSpeed = Math.min(profile.speed(), velocity.getNorm());
      double tangentFF =
          beta < 1 && demand.minus(velocity).dot(tangent.tangent()) <= 1e-3
              ? profile.tangentialAcceleration()
              : 0;
      var ff =
          tangent
              .tangent()
              .times(tangentFF)
              .plus(tangent.normal().times(tangent.curvature() * ffSpeed * ffSpeed));
      telemetry =
          new MotionTelemetry(
              limits.maxSpeed(),
              Math.min(traction, acceleration),
              limits.braking(),
              limits.maxOmega(),
              limits.maxAlpha(),
              profile.speed(),
              profile.tangentialAcceleration(),
              (velocity.getNorm() - oldVelocity.getNorm()) / dt,
              Math.max(0, barrier.progress() - progress),
              distance,
              dwell);
      previous = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
      boolean infeasible = limited.infeasible() || profile.infeasible();
      result =
          new TickResult(
              previous,
              new ChassisAccelerations(ff.getX(), ff.getY(), turn.acceleration()),
              wheelForces(pose, ff, turn.acceleration()),
              status,
              infeasible,
              profile.limitingFactor(),
              limited.limitingFactor(),
              newMarkers == null ? Set.of() : newMarkers,
              immutableStatuses,
              generation,
              projected.progress(),
              diagnostic);
    } catch (IllegalArgumentException invalid) {
      stop(RuntimeStatus.FAULT, invalid.getMessage());
      return brake(pose, snapshot, dt);
    }
    return result;
  }

  private void activate(
      AutoSegment motionPlan, String startAtGoalId, Optional<FieldClearance> clearance) {
    int index = 0;
    while (index < motionPlan.goals().size()
        && !motionPlan.goals().get(index).id().equals(startAtGoalId)) {
      index++;
    }
    if (index == motionPlan.goals().size()) {
      throw new IllegalArgumentException("Unknown starting goal " + startAtGoalId);
    }
    plan = motionPlan;
    boolean dynamic = false;
    for (var goal : motionPlan.goals()) {
      if (goal.dynamicTarget() != null) {
        dynamic = true;
        break;
      }
    }
    hasDynamicGoals = dynamic;
    resolvedGoals = motionPlan.goals();
    fieldClearance = clearance;
    assist = false;
    pendingAssist = false;
    snapHeading = null;
    lastTargets.clear();
    openedGates.clear();
    inheritedHeading = null;
    headingId = null;
    highWater = 0;
    horizonStart = index;
    statuses.clear();
    immutableStatuses = Map.of();
    passed.clear();
    for (var goal : motionPlan.goals().subList(0, index)) {
      if (goal.marker() != null) {
        passed.add(goal.marker());
      }
      if (goal.heading() != null) {
        inheritedHeading = goal.heading().gate(HeadingPlan.GatePolicy.SOFT);
        if (goal.heading().gate() == HeadingPlan.GatePolicy.HARD) {
          openedGates.add(goal.id());
        }
      }
    }
    nominalLimits = DrivetrainLimits.derive(robot, calibration, nominalDt, plan.limits());
    limits = nominalLimits;
    startHorizon();
  }

  private TickResult brake(Pose2d pose, RobotStateSnapshot snapshot, double dt) {
    var old = translation(previous);
    double omega =
        Math.copySign(
            Math.max(0, Math.abs(previous.omegaRadiansPerSecond) - limits.maxAlpha() * dt),
            previous.omegaRadiansPerSecond);
    var limited =
        VectorLimiter.project(
            Translation2d.kZero,
            old,
            pose.getRotation(),
            omega,
            limits.availableTraction(omega, 0),
            limits.acceleration(
                old.getNorm(),
                snapshot.swerveSupplyCurrentLimitAmps(),
                snapshot.restingBatteryVoltageVolts()),
            limits,
            dt);
    var velocity = limited.velocity();
    if (velocity.getNorm() < 1e-6 && Math.abs(omega) < 1e-6) {
      result = zero(stopStatus, stopDiagnostic);
    } else {
      previous = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
      result =
          new TickResult(
              previous,
              new ChassisAccelerations(0, 0, 0),
              List.of(),
              RuntimeStatus.RUNNING,
              limited.infeasible(),
              LimitingFactor.NONE,
              LimitingFactor.VECTOR_SLEW_LIMIT,
              Set.of(),
              immutableStatuses,
              generation,
              highWater,
              stopDiagnostic);
    }
    return result;
  }

  private void checkClearance(
      Pose2d pose, PathBuilder.Path geometry, List<Goal> goals, boolean recovery) {
    if (fieldClearance.isEmpty() || snapHeading != null) {
      return;
    }
    var clearance = fieldClearance.orElseThrow();
    double brake = Math.min(limits.braking(), limits.friction());
    var velocity = translation(previous);
    var arrest = pose.getTranslation().plus(velocity.times(velocity.getNorm() / (2 * brake)));
    double minArrest = Double.POSITIVE_INFINITY;
    int checks = Math.max(1, (int) Math.ceil(pose.getTranslation().getDistance(arrest) / 0.025));
    double startX = pose.getX();
    double startY = pose.getY();
    double arrestDx = arrest.getX() - startX;
    double arrestDy = arrest.getY() - startY;
    double startCos = pose.getRotation().getCos();
    double startSin = pose.getRotation().getSin();
    for (int i = 0; i <= checks; i++) {
      double t = (double) i / checks;
      minArrest =
          Math.min(
              minArrest,
              clearance.clearance(
                  startX + t * arrestDx, startY + t * arrestDy, startCos, startSin));
    }

    double pathLength = geometry.length();
    double startNominal = clearance.clearance(startX, startY, startCos, startSin);
    double prevNominal = startNominal;
    double escapeEndClearance = startNominal;
    boolean escapeNonDecreasing = true;
    double escapeWindow = Math.min(0.15, pathLength);
    boolean checkEscape = startNominal <= 0;
    double required = recovery ? 0 : clearance.margin();
    double safeTarget = Math.max(required, clearance.margin());
    double minPath = minArrest;
    double minBeyondEscape = Double.POSITIVE_INFINITY;
    double bandRadians = Math.toRadians(3.0);
    double cosBand = Math.cos(bandRadians);
    double sinBand = Math.sin(bandRadians);
    double cornerRadius = Math.hypot(clearance.halfLength(), clearance.halfWidth());

    double segStart = 0;
    double currentHeading = pose.getRotation().getRadians();
    boolean hasAnyKeyframe = false;
    for (int goalIdx = 0; goalIdx < goals.size(); goalIdx++) {
      var goal = goals.get(goalIdx);
      var keyframe = goal.heading();
      if (keyframe == null && goalIdx == 0 && inheritedHeading != null) {
        keyframe = inheritedHeading;
      }
      if (keyframe == null && goal.passType() == Goal.PassType.STOP) {
        keyframe = HeadingPlan.to(goal.pose().getRotation());
      }
      if (keyframe == null) {
        continue;
      }
      hasAnyKeyframe = true;
      double anchor =
          MathUtil.clamp(
              geometry.anchors().get(goalIdx) + keyframe.offsetMeters(), segStart, pathLength);
      double spanEnd = goalIdx == goals.size() - 1 ? pathLength : anchor;
      double targetHeadingRad = currentHeading;
      boolean fixedHeading = true;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        targetHeadingRad = fixed.angle().getRadians();
      } else {
        fixedHeading = false;
      }
      double diff = MathUtil.angleModulus(targetHeadingRad - currentHeading);
      if (keyframe.direction() == HeadingPlan.TurnDirection.CW && diff > 0) {
        diff -= 2 * Math.PI;
      } else if (keyframe.direction() == HeadingPlan.TurnDirection.CCW && diff < 0) {
        diff += 2 * Math.PI;
      }
      double spanEndHeading = currentHeading + diff;

      if (fixedHeading) {
        double turn = Math.abs(diff);
        int headingIntervals = (int) Math.ceil(turn / Math.toRadians(5.0));
        double headingAllowance =
            headingIntervals == 0 ? 0.0 : cornerRadius * turn / (2.0 * headingIntervals);
        boolean canDelayTurn = segStart <= 1e-9 && Math.abs(diff) > 1e-6 && anchor > 1e-3;
        double savedMinPath = minPath;
        double savedMinBeyondEscape = minBeyondEscape;
        double savedPrevNominal = prevNominal;
        double savedEscapeEnd = escapeEndClearance;
        boolean savedEscapeNonDecreasing = escapeNonDecreasing;
        spanStateScratch[0] = minPath;
        spanStateScratch[1] = minBeyondEscape;
        spanStateScratch[2] = prevNominal;
        spanStateScratch[3] = escapeEndClearance;
        escapeNonDecreasing =
            evaluateFixedSpan(
                geometry,
                clearance,
                clearanceSampleScratch,
                spanStateScratch,
                escapeNonDecreasing,
                checkEscape,
                escapeWindow,
                required,
                safeTarget,
                segStart,
                anchor,
                spanEnd,
                0.0,
                currentHeading,
                startCos,
                startSin,
                diff,
                spanEndHeading,
                headingIntervals,
                headingAllowance);
        minPath = spanStateScratch[0];
        minBeyondEscape = spanStateScratch[1];
        prevNominal = spanStateScratch[2];
        escapeEndClearance = spanStateScratch[3];

        if (canDelayTurn) {
          boolean immediateEscaping =
              checkEscape
                  && startNominal >= -0.03
                  && escapeNonDecreasing
                  && escapeEndClearance > Math.max(0.0, startNominal) + 1e-4
                  && minArrest >= -0.03
                  && (spanEnd <= escapeWindow + 1e-9 || minBeyondEscape > 0);
          boolean immediateSafe =
              immediateEscaping || (minPath > 0 && (recovery || minPath >= clearance.margin()));
          if (!immediateSafe) {
            double candidate =
                findEarliestSafeTurnStart(
                    geometry,
                    clearance,
                    clearanceSampleScratch,
                    anchor,
                    spanEnd,
                    currentHeading,
                    diff,
                    headingIntervals,
                    headingAllowance,
                    clearance.margin());
            if (candidate < 0 && recovery) {
              candidate =
                  findEarliestSafeTurnStart(
                      geometry,
                      clearance,
                      clearanceSampleScratch,
                      anchor,
                      spanEnd,
                      currentHeading,
                      diff,
                      headingIntervals,
                      headingAllowance,
                      1e-4);
            }
            if (candidate > 0) {
              spanStateScratch[0] = savedMinPath;
              spanStateScratch[1] = savedMinBeyondEscape;
              spanStateScratch[2] = savedPrevNominal;
              spanStateScratch[3] = savedEscapeEnd;
              boolean delayedNonDecreasing =
                  evaluateFixedSpan(
                      geometry,
                      clearance,
                      clearanceSampleScratch,
                      spanStateScratch,
                      savedEscapeNonDecreasing,
                      checkEscape,
                      escapeWindow,
                      required,
                      safeTarget,
                      segStart,
                      anchor,
                      spanEnd,
                      candidate,
                      currentHeading,
                      startCos,
                      startSin,
                      diff,
                      spanEndHeading,
                      headingIntervals,
                      headingAllowance);
              double delayedMinPath = spanStateScratch[0];
              double delayedMinBeyondEscape = spanStateScratch[1];
              double delayedEscapeEnd = spanStateScratch[3];
              boolean delayedEscaping =
                  checkEscape
                      && startNominal >= -0.03
                      && delayedNonDecreasing
                      && delayedEscapeEnd > Math.max(0.0, startNominal) + 1e-4
                      && minArrest >= -0.03
                      && (spanEnd <= escapeWindow + 1e-9 || delayedMinBeyondEscape > 0);
              boolean delayedSafe =
                  delayedEscaping
                      || (delayedMinPath > 0 && (recovery || delayedMinPath >= clearance.margin()));
              if (delayedSafe) {
                minPath = delayedMinPath;
                minBeyondEscape = delayedMinBeyondEscape;
                prevNominal = spanStateScratch[2];
                escapeEndClearance = delayedEscapeEnd;
                escapeNonDecreasing = delayedNonDecreasing;
                horizonTurnStartProgress = candidate;
                horizonHoldHeadingRadians = currentHeading;
              }
            }
          }
        }
      } else {
        int intervals = Math.max(1, (int) Math.ceil((spanEnd - segStart) / 0.025));
        for (int i = 0; i <= intervals; i++) {
          double s = segStart + ((spanEnd - segStart) * i) / intervals;
          geometry.sampleInto(s, clearanceSampleScratch);
          double headingRad =
              plannedHeadingAtRadians(
                  pose, geometry, goals, inheritedHeading, 0.0, s, headingSampleScratch);
          double cos = Math.cos(headingRad);
          double sin = Math.sin(headingRad);
          double px = clearanceSampleScratch[0];
          double py = clearanceSampleScratch[1];
          double nominal = clearance.clearance(px, py, cos, sin);
          double localMin = nominal;
          if (s > escapeWindow || startNominal > 0) {
            localMin =
                Math.min(
                    localMin,
                    clearance.clearance(
                        px, py, cos * cosBand - sin * sinBand, sin * cosBand + cos * sinBand));
            localMin =
                Math.min(
                    localMin,
                    clearance.clearance(
                        px, py, cos * cosBand + sin * sinBand, sin * cosBand - cos * sinBand));
          }
          if (s <= escapeWindow + 1e-9) {
            if (nominal < prevNominal - 1e-6) {
              escapeNonDecreasing = false;
            }
            prevNominal = nominal;
            escapeEndClearance = nominal;
          } else {
            minBeyondEscape = Math.min(minBeyondEscape, localMin);
          }
          minPath = Math.min(minPath, localMin);
        }
      }
      currentHeading = spanEndHeading;
      segStart = spanEnd;
    }

    if (!hasAnyKeyframe) {
      for (double progress = 0; ; ) {
        geometry.sampleInto(progress, clearanceSampleScratch);
        double px = clearanceSampleScratch[0];
        double py = clearanceSampleScratch[1];
        double pointClearance = clearance.clearance(px, py, startCos, startSin);
        minPath = Math.min(minPath, pointClearance);
        if (progress <= escapeWindow + 1e-9) {
          if (pointClearance < prevNominal - 1e-6) {
            escapeNonDecreasing = false;
          }
          prevNominal = pointClearance;
          escapeEndClearance = pointClearance;
        } else {
          minBeyondEscape = Math.min(minBeyondEscape, pointClearance);
        }
        if (progress >= pathLength - 1e-9) {
          break;
        }
        double remaining = pathLength - progress;
        double step;
        if (checkEscape && progress < escapeWindow - 1e-9) {
          step = Math.min(0.025, Math.min(escapeWindow - progress, remaining));
        } else if (pointClearance > required + 0.02) {
          step = Math.min(0.25, Math.min((pointClearance - required) / 2.0, remaining));
          minPath = Math.min(minPath, pointClearance - step);
          if (progress > escapeWindow + 1e-9) {
            minBeyondEscape = Math.min(minBeyondEscape, pointClearance - step);
          }
        } else {
          step = Math.min(0.025, remaining);
        }
        progress = Math.min(pathLength, progress + Math.max(step, 1e-6));
      }
    }

    if (pathLength <= escapeWindow + 1e-9) {
      minBeyondEscape = escapeEndClearance;
    }
    boolean escapingWall =
        startNominal <= 0
            && startNominal >= -0.03
            && escapeNonDecreasing
            && escapeEndClearance > Math.max(0.0, startNominal) + 1e-4
            && minArrest >= -0.03
            && minBeyondEscape > 0;
    if (!escapingWall && (minPath <= 0 || (!recovery && minPath < clearance.margin()))) {
      throw new IllegalArgumentException("Path footprint intersects a keepout or required margin");
    }
    marginWarning = minPath < clearance.margin();
    limits =
        marginWarning
            ? DrivetrainLimits.derive(robot, calibration, nominalDt, plan.limits(), 0.60)
            : nominalLimits;
  }

  private List<RotationBudgetEstimator.HeadingSpan> collectDownstreamHeadingSpans(
      double progress, double barrierProgress) {
    if (horizon == null || path == null) {
      return List.of();
    }
    List<RotationBudgetEstimator.HeadingSpan> spans = null;
    boolean appliedDelay = false;
    double spanStart = progress;
    double prevAngle = targetHeading;
    for (int i = goalIndex; i < horizon.size(); i++) {
      var goal = resolvedGoals.get(horizonStart + i);
      var keyframe = goal.heading();
      if (keyframe == null && goal.passType() == Goal.PassType.STOP) {
        keyframe = HeadingPlan.to(goal.pose().getRotation());
      }
      if (keyframe == null) {
        continue;
      }
      if (!appliedDelay) {
        appliedDelay = true;
        if (horizonTurnStartProgress > spanStart && headingId == null) {
          spanStart = Math.min(barrierProgress, horizonTurnStartProgress);
        }
      }
      double anchor =
          MathUtil.clamp(
              path.anchors().get(i) + keyframe.offsetMeters(), spanStart, barrierProgress);
      if (goal.id().equals(headingId)) {
        spanStart = anchor;
        continue;
      }
      double targetAngle = prevAngle;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        double diff = MathUtil.angleModulus(fixed.angle().getRadians() - prevAngle);
        if (keyframe.direction() == HeadingPlan.TurnDirection.CW && diff > 0) {
          diff -= 2 * Math.PI;
        } else if (keyframe.direction() == HeadingPlan.TurnDirection.CCW && diff < 0) {
          diff += 2 * Math.PI;
        }
        targetAngle = prevAngle + diff;
      } else if (keyframe.target() instanceof HeadingPlan.Target.Travel) {
        var sample = path.sample(anchor);
        if (sample.tangent().getNorm() > 1e-6) {
          targetAngle =
              prevAngle
                  + MathUtil.angleModulus(sample.tangent().getAngle().getRadians() - prevAngle);
        }
      } else if (keyframe.target() instanceof HeadingPlan.Target.Point fixed) {
        var pos = path.sample(anchor).position();
        var delta = fixed.target().minus(pos);
        if (delta.getNorm() >= 0.25) {
          double aim = delta.getAngle().getRadians() - fixed.mechanismOffset().getRadians();
          targetAngle = prevAngle + MathUtil.angleModulus(aim - prevAngle);
        }
      }
      double delta = targetAngle - prevAngle;
      if (Math.abs(delta) > 0.01) {
        if (spans == null) {
          spans = new ArrayList<>();
        }
        spans.add(new RotationBudgetEstimator.HeadingSpan(spanStart, anchor, delta));
      }
      prevAngle = targetAngle;
      spanStart = anchor;
    }
    return spans == null ? List.of() : spans;
  }

  private Barrier headingBarrier(
      TickInput input,
      Pose2d pose,
      ChassisSpeeds measured,
      PathBuilder.Projection projection,
      double dt) {
    double activation = 0;
    double terminal = path.anchors().get(horizon.size() - 1);
    targetDerivative = 0;
    faceTravel = false;
    singularAim = false;
    if (horizonTurnStartProgress > 0
        && highWater < horizonTurnStartProgress - 1e-4
        && projection.progress() < horizonTurnStartProgress - 1e-4) {
      targetHeading =
          heading.angle() + MathUtil.angleModulus(horizonHoldHeadingRadians - heading.angle());
      headingId = null;
      headingDwell = 0;
      gateWait = 0;
      return nextClosedGate(0, 0, terminal);
    }
    for (int i = 0; i < horizon.size(); i++) {
      var goal = resolvedGoals.get(horizonStart + i);
      var keyframe = goal.heading();
      boolean usingInherited = false;
      if (keyframe == null && goal.passType() == Goal.PassType.STOP) {
        keyframe = HeadingPlan.to(goal.pose().getRotation());
      }
      if (keyframe == null && i == 0 && inheritedHeading != null) {
        keyframe = inheritedHeading;
        usingInherited = true;
      }
      if (keyframe == null) {
        continue;
      }
      if (!usingInherited) {
        inheritedHeading = null;
      }
      double anchor =
          MathUtil.clamp(path.anchors().get(i) + keyframe.offsetMeters(), activation, terminal);
      if (projection.progress() < activation) {
        break;
      }
      if (projection.progress() > anchor
          && i < horizon.size() - 1
          && (keyframe.gate() != HeadingPlan.GatePolicy.HARD || openedGates.contains(goal.id()))) {
        activation = anchor;
        continue;
      }
      var sample = projection.sample();
      boolean entering = !goal.id().equals(headingId);
      double raw = entering ? pose.getRotation().getRadians() : targetHeading;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        raw = fixed.angle().getRadians();
      } else if (keyframe.target() instanceof HeadingPlan.Target.Travel) {
        faceTravel = true;
        singularAim = sample.tangent().getNorm() == 0;
        if (!singularAim) {
          raw = sample.tangent().getAngle().getRadians();
          targetDerivative = sample.curvature();
        }
      } else if (keyframe.target() instanceof HeadingPlan.Target.Point
          || keyframe.target() instanceof HeadingPlan.Target.DynamicPoint) {
        Translation2d aim;
        Rotation2d offset;
        if (keyframe.target() instanceof HeadingPlan.Target.Point fixed) {
          aim = fixed.target();
          offset = fixed.mechanismOffset();
        } else {
          var dynamic = (HeadingPlan.Target.DynamicPoint) keyframe.target();
          var target =
              input.resolvedAimTargets().containsKey(goal.id())
                  ? input.resolvedAimTargets().get(goal.id())
                  : dynamic.supplier().get();
          aim = target == null || !target.fresh(input.timestampSeconds()) ? null : target.value();
          offset = dynamic.mechanismOffset();
        }
        var delta = aim == null ? Translation2d.kZero : aim.minus(sample.position());
        double range = delta.getNorm();
        singularAim = !Double.isFinite(range) || range < 0.25;
        if (!singularAim) {
          raw = delta.getAngle().getRadians() - offset.getRadians();
          targetDerivative =
              MathUtil.clamp((range - 0.25) / 0.15, 0, 1)
                  * (-delta.getX() * sample.tangent().getY()
                      + delta.getY() * sample.tangent().getX())
                  / (range * range);
        }
      }
      if (entering) {
        double difference = MathUtil.angleModulus(raw - heading.angle());
        if (keyframe.direction() == HeadingPlan.TurnDirection.CW && difference > 0) {
          difference -= 2 * Math.PI;
        } else if (keyframe.direction() == HeadingPlan.TurnDirection.CCW && difference < 0) {
          difference += 2 * Math.PI;
        }
        targetHeading = heading.angle() + difference;
        headingId = goal.id();
        headingDwell = 0;
        gateWait = 0;
      } else {
        targetHeading += MathUtil.angleModulus(raw - targetHeading);
      }
      if (keyframe.gate() == HeadingPlan.GatePolicy.HARD) {
        if (openedGates.contains(goal.id()) && projection.progress() < anchor) {
          double dropoutError =
              Math.abs(MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()));
          if (dropoutError > 2.0 * keyframe.toleranceRadians()) {
            openedGates.remove(goal.id());
            headingDwell = 0;
          }
        }
        if (!openedGates.contains(goal.id())) {
          double distance = pose.getTranslation().getDistance(path.sample(anchor).position());
          boolean ready =
              !singularAim
                  && distance <= 0.25
                  && Math.abs(
                          MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()))
                      <= keyframe.toleranceRadians()
                  && Math.abs(measured.omegaRadiansPerSecond) <= keyframe.angularSpeedTolerance()
                  && Math.abs(targetHeading - heading.angle()) < Math.PI / 2;
          headingDwell = ready ? headingDwell + dt : 0;
          if (ready && headingDwell + 1e-9 >= keyframe.dwellSeconds()) {
            openedGates.add(goal.id());
          } else {
            gateWait = distance <= 0.25 ? gateWait + dt : 0;
            if ((projection.progress() > anchor && distance > 0.25) || gateWait > 3) {
              stop(RuntimeStatus.FAULT, "Missed or stalled heading gate " + goal.id());
            }
            return new Barrier(anchor, true);
          }
        }
      }
      if (projection.progress() <= anchor || i == horizon.size() - 1) {
        return nextClosedGate(i + 1, anchor, terminal);
      }
      activation = anchor;
    }
    return new Barrier(path.length(), false);
  }

  private TickResult holdDuringSnapshotGrace(double dt) {
    var oldVelocity = previous == null ? Translation2d.kZero : translation(previous);
    double prevOmega = previous == null ? 0.0 : previous.omegaRadiansPerSecond;
    double decel =
        limits == null
            ? calibration.maxBrakingDeceleration()
            : Math.min(limits.braking(), limits.friction());
    double maxAlpha = limits == null ? 4.0 : limits.maxAlpha();
    double oldSpeed = oldVelocity.getNorm();
    var nextVelocity =
        oldSpeed > 1e-9
            ? oldVelocity.times(Math.max(0.0, 1.0 - Math.min(oldSpeed, decel * dt) / oldSpeed))
            : Translation2d.kZero;
    double nextOmega = Math.copySign(Math.max(0.0, Math.abs(prevOmega) - maxAlpha * dt), prevOmega);
    previous = new ChassisSpeeds(nextVelocity.getX(), nextVelocity.getY(), nextOmega);
    return new TickResult(
        previous,
        new ChassisAccelerations(0, 0, 0),
        List.of(),
        RuntimeStatus.RUNNING,
        false,
        LimitingFactor.NONE,
        LimitingFactor.VECTOR_SLEW_LIMIT,
        Set.of(),
        immutableStatuses,
        generation,
        highWater,
        "Waiting for fresh odometry snapshot");
  }

  private Barrier nextClosedGate(int from, double activation, double terminal) {
    for (int i = from; i < horizon.size(); i++) {
      var goal = horizon.get(i);
      var keyframe = goal.heading();
      if (keyframe == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(path.anchors().get(i) + keyframe.offsetMeters(), activation, terminal);
      if (keyframe.gate() == HeadingPlan.GatePolicy.HARD && !openedGates.contains(goal.id())) {
        return new Barrier(anchor, true);
      }
      activation = anchor;
    }
    return new Barrier(path.length(), false);
  }

  private void rebuild(Pose2d pose, int from) {
    int end = from;
    while (end < resolvedGoals.size() - 1
        && resolvedGoals.get(end).passType() == Goal.PassType.THROUGH) {
      end++;
    }
    marginWarning = false;
    limits = nominalLimits;
    horizonTurnStartProgress = 0.0;
    horizonHoldHeadingRadians = pose.getRotation().getRadians();
    var candidate = resolvedGoals.subList(from, end + 1);
    var geometry =
        PathBuilder.build(
            pose.getTranslation(),
            candidate,
            Math.min(limits.braking(), limits.friction()),
            fieldClearance.orElse(null));
    if (geometry.length() > 100) {
      throw new IllegalArgumentException("Path exceeds profile capacity");
    }
    checkClearance(pose, geometry, candidate, path != null && !assist);
    horizonStart = from;
    horizon = List.copyOf(candidate);
    rebuildStartPose = pose;
    geometryTargets.clear();
    for (var goal : candidate) {
      geometryTargets.put(goal.id(), goal.pose());
    }
    path = geometry;
    goalIndex = 0;
    generation++;
    highWater = 0;
    closest = Double.POSITIVE_INFINITY;
    offTrack = 0;
    dwell = 0;
    terminalLatched = false;
    lastPosition = pose.getTranslation();
  }

  private Set<Enum<?>> recordPass(Goal goal, boolean within, Set<Enum<?>> newMarkers) {
    statuses.put(
        goal.id(),
        within
            ? GoalPassageStatus.PASSED_WITHIN_TOLERANCE
            : GoalPassageStatus.PASSED_OUTSIDE_TOLERANCE);
    immutableStatuses = Map.copyOf(statuses);
    if (goal.marker() != null && passed.add(goal.marker())) {
      if (newMarkers == null) {
        newMarkers = new LinkedHashSet<>();
      }
      newMarkers.add(goal.marker());
    }
    return newMarkers;
  }

  private boolean resolveGoals(TickInput input, int from) {
    if (!hasDynamicGoals) {
      resolvedGoals = plan.goals();
      return true;
    }
    var resolved = new ArrayList<>(plan.goals());
    for (int index = from; index < resolved.size(); index++) {
      var goal = resolved.get(index);
      var dynamic = goal.dynamicTarget();
      if (dynamic != null) {
        var target =
            input.resolvedGoalTargets().containsKey(goal.id())
                ? input.resolvedGoalTargets().get(goal.id())
                : dynamic.supplier().get();
        double now = input.timestampSeconds();
        Pose2d value;
        if (target != null
            && target.fresh(now)
            && Double.isFinite(target.value().getTranslation().getNorm())
            && Double.isFinite(target.value().getRotation().getRadians())) {
          lastTargets.put(goal.id(), target);
          value = target.value();
        } else if (dynamic.lossPolicy() instanceof TargetLossPolicy.FallbackNominal fallback) {
          value = fallback.nominalPose();
        } else if (dynamic.lossPolicy() instanceof TargetLossPolicy.HoldLastValid hold
            && lastTargets.containsKey(goal.id())
            && now - lastTargets.get(goal.id()).timestampSeconds() <= hold.graceSeconds()) {
          value = lastTargets.get(goal.id()).value();
        } else {
          return false;
        }
        resolved.set(index, goal.withPose(value));
      }
      if (goal.passType() != Goal.PassType.THROUGH) {
        break;
      }
    }
    resolvedGoals = List.copyOf(resolved);
    return true;
  }

  private void startHorizon() {
    path = null;
    telemetry = null;
    rebuildStartPose = Pose2d.kZero;
    marginWarning = false;
    horizonTurnStartProgress = 0.0;
    horizonHoldHeadingRadians = 0.0;
    unhealthySnapshotSeconds = 0;
    lastTime = Double.NaN;
    activationTime = Double.NaN;
    stopping = false;
    headingDwell = 0;
    gateWait = 0;
    result = zero(RuntimeStatus.RUNNING, "");
  }

  private void stop(RuntimeStatus status, String diagnostic) {
    stopping = true;
    stopStatus = status;
    stopDiagnostic = diagnostic;
  }

  private boolean targetsMoved() {
    for (int i = goalIndex; i < horizon.size(); i++) {
      var goal = resolvedGoals.get(horizonStart + i);
      if (geometryTargets.get(goal.id()).getTranslation().getDistance(goal.pose().getTranslation())
          > 0.01) {
        return true;
      }
    }
    return false;
  }

  private List<Translation2d> wheelForces(Pose2d pose, Translation2d acceleration, double alpha) {
    var modules = robot.modulePositionsMeters();
    if (modules.isEmpty()) {
      return List.of();
    }
    double torque = robot.yawMoiKgMetersSq() * alpha / squaredModuleRadii;
    double cap = robot.massKg() * limits.friction() / 4;
    double massQuarter = robot.massKg() / 4;
    double baseFx = acceleration.getX() * massQuarter;
    double baseFy = acceleration.getY() * massQuarter;
    double cos = pose.getRotation().getCos();
    double sin = pose.getRotation().getSin();
    Translation2d[] forces = new Translation2d[modules.size()];
    for (int i = 0; i < modules.size(); i++) {
      var m = modules.get(i);
      double rx = m.getX() * cos - m.getY() * sin;
      double ry = m.getX() * sin + m.getY() * cos;
      double fx = baseFx - ry * torque;
      double fy = baseFy + rx * torque;
      double norm = Math.hypot(fx, fy);
      if (norm > cap) {
        double scale = cap / norm;
        fx *= scale;
        fy *= scale;
      }
      forces[i] = new Translation2d(fx, fy);
    }
    return List.of(forces);
  }

  private TickResult zero(RuntimeStatus status, String diagnostic) {
    return new TickResult(
        new ChassisSpeeds(),
        new ChassisAccelerations(0, 0, 0),
        List.of(),
        status,
        false,
        LimitingFactor.NONE,
        LimitingFactor.NONE,
        Set.of(),
        immutableStatuses,
        generation,
        highWater,
        diagnostic);
  }
}
