package com.team581.trailblazer3.runtime;

import static com.google.common.base.Preconditions.checkArgument;
import static com.team581.trailblazer3.math.HeadingSchedule.turnDelta;
import static java.util.Objects.requireNonNull;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.math.DrivetrainLimits;
import com.team581.trailblazer3.math.HeadingPlanner;
import com.team581.trailblazer3.math.HeadingSchedule;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.math.RotationBudgetEstimator;
import com.team581.trailblazer3.math.VectorLimiter;
import com.team581.trailblazer3.math.VelocityProfiler;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

/** Physics-limited sequential path follower for static autonomous segments. */
public final class Trailblazer {
  /** Measured heading error at a through point's heading anchor. */
  public record HeadingArrival(int goalIndex, double errorRadians, boolean withinTolerance) {}

  private record Barrier(double progress, boolean closed) {}

  private record HeadingPassage(int goal, double progress, HeadingPlan keyframe) {}

  private static final double CORRIDOR = 0.30;
  private static final double RELATCH_DWELL = 0.15;
  private static final double MAX_TICK_INTERVAL = 0.10;
  private static final TickResult IDLE_RESULT =
      new TickResult(
          new ChassisSpeeds(), RuntimeStatus.IDLE, false, LimitingFactor.NONE, 0, 0.0, "");

  /** Samples discrete poses along a path geometry with planned per-goal headings for previews. */
  public static Pose2d[] sampleSegmentPoses(
      Pose2d startPose, PathBuilder.Path geometry, List<Goal> goals, double stepMeters) {
    if (geometry == null
        || goals == null
        || goals.isEmpty()
        || !Double.isFinite(stepMeters)
        || stepMeters <= 0) {
      return new Pose2d[0];
    }
    return sampleSegmentPoses(startPose, geometry, goals, 0.0, stepMeters, new double[5]);
  }

  /** Starts a named segment with the robot’s default motion limits. */
  public static AutoSegment.Builder segment(String id) {
    return new AutoSegment.Builder(id);
  }

  private static boolean finite(ChassisSpeeds speed) {
    return speed != null
        && Double.isFinite(speed.vxMetersPerSecond)
        && Double.isFinite(speed.vyMetersPerSecond)
        && Double.isFinite(speed.omegaRadiansPerSecond);
  }

  private static boolean finite(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  private static HeadingPlan headingForGoal(Goal goal) {
    if (goal.heading() != null) {
      return goal.heading();
    }
    return goal.passType() == Goal.PassType.STOP ? HeadingPlan.to(goal.pose().getRotation()) : null;
  }

  private static double plannedHeadingAtRadians(
      Pose2d startPose,
      PathBuilder.Path geometry,
      List<Goal> goals,
      double firstTurnStartProgress,
      double s,
      double[] sampleScratch) {
    double segStart = 0;
    double current = startPose.getRotation().getRadians();
    boolean appliedDelay = false;
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      var keyframe = headingForGoal(goal);
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
        double diff = turnDelta(current, target, keyframe.direction());
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
              startPose, geometry, goals, firstTurnStartProgress, s, sampleScratch);
      poses[i] = new Pose2d(px, py, Rotation2d.fromRadians(headingRad));
    }
    return poses;
  }

  private static boolean sweepClear(
      FieldClearance clearance,
      double x,
      double y,
      double startHeading,
      double diff,
      int angleSteps) {
    for (int step = 0; step <= angleSteps; step++) {
      double angle = startHeading + (diff * step) / angleSteps;
      if (clearance.clearance(x, y, Math.cos(angle), Math.sin(angle)) < clearance.margin()) {
        return false;
      }
    }
    return true;
  }

  private static Translation2d translation(ChassisSpeeds speed) {
    return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
  }

  private final DoubleSupplier clock;
  private final RobotDrivetrainSpec robot;
  private final EmpiricalDriveCalibration calibration;
  private final double nominalDt;
  private final Optional<FieldClearance> defaultFieldClearance;
  private final Set<Enum<?>> passed = new LinkedHashSet<>();
  private final VelocityProfiler.Workspace profilerWorkspace = new VelocityProfiler.Workspace();
  private final double[] headingSampleScratch = new double[5];
  private final List<HeadingArrival> headingArrivals = new ArrayList<>();

  private AutoSegment plan;
  private Optional<FieldClearance> activeClearance = Optional.empty();
  private DrivetrainLimits limits;
  private List<Goal> activeGoals = ImmutableList.of();
  private boolean[] openedGates = new boolean[0];
  private PathBuilder.Path path;
  private int remainingGoalStart;
  private int goalIndex;
  private long generation;
  private double highWater;
  private double dwell;
  private double offTrack;
  private double divergence;
  private double lastTime = Double.NaN;
  private boolean relatchRequested;

  private int headingGoalIndex = -1;
  private double headingDwell;
  private double gateWait;
  private double targetHeading;
  private double targetDerivative;
  private double referenceAlpha;
  private boolean faceTravel;
  private List<HeadingSchedule> pathHeadings = ImmutableList.of();
  private List<HeadingPassage> headingPassages = ImmutableList.of();
  private int nextHeadingPassage;
  private double previousHeadingProgress;
  private Rotation2d previousMeasuredHeading = Rotation2d.kZero;
  private double turnHoldProgress;
  private double turnHoldHeadingRadians;

  private boolean terminalLatched;
  private boolean stopPositionLatched;
  private boolean stopHeadingLatched;

  private Pose2d rebuildStartPose = Pose2d.kZero;
  private HeadingPlanner.State heading;
  private ChassisSpeeds previous;
  private TickResult result = IDLE_RESULT;

  /** Creates a follower with a nominal period in seconds and no default field clearance. */
  public Trailblazer(
      DoubleSupplier clock,
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt) {
    this(clock, robot, calibration, nominalDt, Optional.empty());
  }

  /** Creates a follower with a nominal period in seconds and optional default field clearance. */
  public Trailblazer(
      DoubleSupplier clock,
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      Optional<FieldClearance> defaultFieldClearance) {
    this.clock = requireNonNull(clock);
    this.robot = requireNonNull(robot);
    this.calibration = requireNonNull(calibration);
    this.defaultFieldClearance = requireNonNull(defaultFieldClearance);
    checkArgument(
        Double.isFinite(nominalDt) && nominalDt >= 0.004 && nominalDt <= 0.020,
        "Nominal period must be within 4–20 ms");
    this.nominalDt = nominalDt;
  }

  /** Starts a segment with the default field clearance and clears previous marker state. */
  public void activate(AutoSegment motionPlan) {
    activate(motionPlan, defaultFieldClearance);
  }

  /** Starts a segment with an explicit field clearance model. */
  public void activate(AutoSegment motionPlan, FieldClearance clearance) {
    activate(motionPlan, Optional.of(requireNonNull(clearance)));
  }

  /** Starts a segment with field clearance checks disabled. */
  public void activateWithoutClearance(AutoSegment motionPlan) {
    activate(motionPlan, Optional.empty());
  }

  /** Reports whether the current segment has reached its terminal completion condition. */
  public boolean atGoal() {
    return result.status() == RuntimeStatus.AT_GOAL;
  }

  /** Clears the active segment, marker state, and drive command. */
  public void cancel() {
    plan = null;
    path = null;
    activeGoals = ImmutableList.of();
    openedGates = new boolean[0];
    passed.clear();
    previous = null;
    heading = null;
    lastTime = Double.NaN;
    relatchRequested = false;
    result =
        new TickResult(
            new ChassisSpeeds(), RuntimeStatus.IDLE, false, LimitingFactor.NONE, generation, 0, "");
  }

  /**
   * Returns the path position at recorded progress and the active target heading, when available.
   */
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

  /** Returns heading passage measurements for the current segment, in passage order. */
  public ImmutableList<HeadingArrival> headingArrivals() {
    return ImmutableList.copyOf(headingArrivals);
  }

  /** Returns the most recent stored follower result. */
  public TickResult lastResult() {
    return result;
  }

  /** Reads the follower clock in seconds. */
  public double nowSeconds() {
    return clock.getAsDouble();
  }

  /** Reports whether the active segment has recorded the given goal marker. */
  public boolean passedMarker(Enum<?> marker) {
    return passed.contains(marker);
  }

  /** Requests rebuilding the remaining path from the measured pose on the next update. */
  public void relatch() {
    relatchRequested = true;
  }

  /** Samples the built path for preview at the given spacing in meters. */
  public Pose2d[] sampledPathPoses(double stepMeters) {
    if (path == null || activeGoals.isEmpty() || !Double.isFinite(stepMeters) || stepMeters <= 0) {
      return new Pose2d[0];
    }
    return sampleSegmentPoses(
        rebuildStartPose, path, activeGoals, turnHoldProgress, stepMeters, headingSampleScratch);
  }

  /**
   * Computes a drive command using an explicit timestamp in seconds and field-relative
   * measurements.
   */
  public TickResult update(
      double nowSeconds, Pose2d measuredPose, ChassisSpeeds measuredFieldSpeeds) {
    if (plan == null) {
      result =
          new TickResult(
              new ChassisSpeeds(),
              RuntimeStatus.IDLE,
              false,
              LimitingFactor.NONE,
              generation,
              0,
              "");
      return result;
    }
    if (atGoal() && activeGoals.get(activeGoals.size() - 1).passType() == Goal.PassType.STOP) {
      return result;
    }
    if (!Double.isFinite(nowSeconds) || !finite(measuredPose) || !finite(measuredFieldSpeeds)) {
      previous = null;
      lastTime = Double.NaN;
      return new TickResult(
          new ChassisSpeeds(),
          RuntimeStatus.RUNNING,
          false,
          LimitingFactor.NONE,
          generation,
          highWater,
          "NONFINITE_STATE");
    }

    boolean firstTick = Double.isNaN(lastTime);
    if (!firstTick && nowSeconds >= lastTime - 1e-4 && nowSeconds - lastTime < 1e-4) {
      return result;
    }
    boolean missedTick =
        !firstTick && (nowSeconds < lastTime - 1e-4 || nowSeconds - lastTime > MAX_TICK_INTERVAL);
    double dt = firstTick || missedTick ? nominalDt : (nowSeconds - lastTime);
    lastTime = nowSeconds;

    boolean reseed = missedTick || previous == null;
    if (reseed) {
      previous = measuredFieldSpeeds;
    }
    if (path == null || reseed) {
      double measuredHeading = measuredPose.getRotation().getRadians();
      if (heading != null) {
        measuredHeading =
            heading.angle() + MathUtil.angleModulus(measuredHeading - heading.angle());
      }
      heading = new HeadingPlanner.State(measuredHeading, previous.omegaRadiansPerSecond);
      referenceAlpha = 0.0;
    }
    if (path == null) {
      headingGoalIndex = -1;
      targetHeading = heading.angle();
    }
    if (missedTick) {
      dwell = 0;
      headingDwell = 0;
      offTrack = 0;
      divergence = 0;
      stopPositionLatched = false;
      stopHeadingLatched = false;
    }

    String diagnostic = missedTick ? "MISSED_TICK_RECOVERY" : "";
    if (path == null) {
      rebuild(measuredPose, remainingGoalStart);
    } else if (relatchRequested) {
      relatchRequested = false;
      int lastGoalIndex = Math.max(0, activeGoals.size() - 1);
      rebuild(measuredPose, remainingGoalStart + Math.min(goalIndex, lastGoalIndex));
      if (diagnostic.isEmpty()) {
        diagnostic = "RELATCH";
      }
    }

    double firstAllowed = goalIndex > 1 ? path.anchors().get(goalIndex - 2) : 0;
    double lastAllowed =
        goalIndex + 1 < activeGoals.size() ? path.anchors().get(goalIndex + 1) : path.length();
    var projected =
        path.project(
            measuredPose.getTranslation(),
            Math.max(firstAllowed, highWater - 0.60),
            Math.min(lastAllowed, highWater + 1.50));
    var measured = translation(measuredFieldSpeeds);
    offTrack =
        projected.distance() > CORRIDOR || highWater - projected.progress() > 0.55
            ? offTrack + dt
            : 0;
    divergence = measured.getDistance(translation(previous)) > 1.0 ? divergence + dt : 0;
    if (divergence >= RELATCH_DWELL) {
      previous = measuredFieldSpeeds;
      divergence = 0;
    }
    if (offTrack >= RELATCH_DWELL && !atGoal()) {
      int lastGoalIndex = Math.max(0, activeGoals.size() - 1);
      rebuild(measuredPose, remainingGoalStart + Math.min(goalIndex, lastGoalIndex));
      projected = path.project(measuredPose.getTranslation(), 0, 0);
      diagnostic = "RELATCH";
    }

    var barrier = headingBarrier(measuredPose, measuredFieldSpeeds, projected, dt);
    if (gateWait == -1.0) {
      gateWait = 0.0;
      if (diagnostic.isEmpty()) {
        diagnostic = "HARD_GATE_BYPASSED";
      }
    }
    if (Math.abs(projected.crossTrack()) <= CORRIDOR && projected.distance() <= CORRIDOR) {
      double progressBeforeBarrier = Math.min(projected.progress(), barrier.progress());
      highWater = Math.max(highWater, progressBeforeBarrier);
      if (recordHeadingArrivals(progressBeforeBarrier, measuredPose.getRotation(), barrier)
          && diagnostic.isEmpty()) {
        diagnostic = "HEADING_TOLERANCE_MISS";
      }
    }

    RuntimeStatus status = atGoal() ? RuntimeStatus.AT_GOAL : RuntimeStatus.RUNNING;
    while (goalIndex < activeGoals.size()) {
      var goal = activeGoals.get(goalIndex);
      if (goal.passType() == Goal.PassType.STOP) {
        break;
      }
      if (highWater + 1e-9 < path.anchors().get(goalIndex)
          || (barrier.closed() && path.anchors().get(goalIndex) >= barrier.progress() - 1e-9)) {
        break;
      }
      if (goal.marker() != null) {
        passed.add(goal.marker());
      }
      goalIndex++;
      if (goal.passType() == Goal.PassType.FINISH_THROUGH) {
        status = RuntimeStatus.AT_GOAL;
      }
    }

    var terminal = activeGoals.get(activeGoals.size() - 1);
    double progress = projected.progress();
    double along = translation(previous).dot(projected.sample().tangent());
    var downstreamSpans = collectDownstreamHeadingSpans(progress, barrier.progress());
    HeadingSchedule spatial = null;
    for (var section : pathHeadings) {
      if (progress >= section.start() && progress < section.end() && !barrier.closed()) {
        spatial = section;
        break;
      }
    }
    var profile =
        profileTranslation(
            progress, along, barrier.progress(), downstreamSpans, spatial != null, dt);
    double remainingToBarrier = Math.max(0.0, barrier.progress() - progress);
    double brakingSpeed =
        Math.sqrt(2.0 * Math.min(limits.braking(), limits.friction()) * remainingToBarrier);
    double cappedBrakingSpeed = Math.min(limits.maxSpeed(), brakingSpeed);
    double headingBudgetSpeed = Math.max(profile.speed(), cappedBrakingSpeed);

    var tolerance = terminal.finishTolerance();
    boolean stopTerminalWindow =
        terminal.passType() == Goal.PassType.STOP
            && !barrier.closed()
            && goalIndex >= activeGoals.size() - 1
            && path.length() - progress <= Math.max(CORRIDOR, tolerance.positionMeters() + 0.05);
    double headingError =
        Math.abs(MathUtil.angleModulus(targetHeading - measuredPose.getRotation().getRadians()));
    if (stopTerminalWindow
        && Math.abs(targetDerivative * profile.speed()) <= 1e-3
        && Math.abs(targetHeading - heading.angle()) < Math.PI / 2) {
      stopHeadingLatched =
          headingError <= (stopHeadingLatched ? 1.0 : 0.5) * tolerance.headingRadians()
              || (headingError <= tolerance.headingRadians()
                  && Math.abs(measuredFieldSpeeds.omegaRadiansPerSecond)
                      <= tolerance.angularSpeed());
    } else {
      stopHeadingLatched = false;
    }

    double omega;
    if (stopHeadingLatched) {
      omega =
          MathUtil.clamp(
              0.0,
              previous.omegaRadiansPerSecond - limits.maxAlpha() * dt,
              previous.omegaRadiansPerSecond + limits.maxAlpha() * dt);
      referenceAlpha = 0.0;
      double unwrappedMeasured =
          targetHeading
              + MathUtil.angleModulus(measuredPose.getRotation().getRadians() - targetHeading);
      heading = new HeadingPlanner.State(unwrappedMeasured, 0.0);
    } else if (spatial != null) {
      var turn =
          HeadingPlanner.track(
              spatial,
              progress,
              measured.dot(projected.sample().tangent()),
              along,
              measuredPose,
              measuredFieldSpeeds.omegaRadiansPerSecond,
              previous.omegaRadiansPerSecond,
              limits,
              dt);
      heading = turn.reference();
      referenceAlpha = turn.acceleration();
      omega = turn.command();
    } else {
      var turn =
          HeadingPlanner.step(
              heading,
              targetHeading,
              targetDerivative,
              measuredPose.getRotation().getRadians(),
              previous.omegaRadiansPerSecond,
              profile.speed(),
              headingBudgetSpeed,
              limits,
              dt);
      heading = turn.reference();
      referenceAlpha = turn.acceleration();
      omega = turn.command();
    }

    double traction = limits.availableTraction(heading.velocity(), referenceAlpha);
    var tangent = path.sample(profile.tangentProgress());
    double tau = Math.max(0.12, 4 * (limits.lag() + nominalDt / 2));
    double crossTrackCap = Math.min(CORRIDOR / tau, 0.30 * limits.maxSpeed());
    var demand =
        tangent
            .tangent()
            .times(profile.speed())
            .plus(
                tangent
                    .normal()
                    .times(
                        MathUtil.clamp(
                            -projected.crossTrack() / tau, -crossTrackCap, crossTrackCap)));
    var error = path.sample(barrier.progress()).position().minus(measuredPose.getTranslation());
    double distance = error.getNorm();
    double barrierGap = barrier.progress() - progress;
    double goalDistance =
        measuredPose.getTranslation().getDistance(terminal.pose().getTranslation());
    if (stopTerminalWindow) {
      stopPositionLatched =
          goalDistance <= (stopPositionLatched ? 1.0 : 0.5) * tolerance.positionMeters()
              || (goalDistance <= tolerance.positionMeters()
                  && measured.getNorm() <= tolerance.linearSpeed());
    } else {
      stopPositionLatched = false;
    }
    terminalLatched =
        stopPositionLatched
            || (barrierGap <= (terminalLatched ? 0.30 : 0.25)
                && distance <= (terminalLatched ? 0.12 : 0.08));
    double beta =
        terminalLatched || barrierGap <= 0
            ? 1
            : MathUtil.clamp((0.25 - Math.max(barrierGap, distance)) / 0.17, 0, 1);
    double terminalSpeed =
        stopPositionLatched
            ? 0.0
            : Math.min(
                limits.maxSpeed(),
                Math.min(
                    Math.sqrt(2 * Math.min(limits.braking(), traction) * distance),
                    distance / tau));
    var terminalDemand =
        distance > 1e-9 && terminalSpeed > 0.0
            ? error.times(terminalSpeed / distance)
            : Translation2d.kZero;
    demand = demand.times(1 - beta).plus(terminalDemand.times(beta));

    var oldVelocity = translation(previous);
    double acceleration = limits.acceleration(oldVelocity.getNorm());
    var limited =
        VectorLimiter.project(
            demand,
            oldVelocity,
            measuredPose.getRotation(),
            omega,
            traction,
            acceleration,
            limits,
            dt);
    if (limited.infeasible() && omega != previous.omegaRadiansPerSecond) {
      var retry =
          VectorLimiter.project(
              demand,
              oldVelocity,
              measuredPose.getRotation(),
              previous.omegaRadiansPerSecond,
              traction,
              acceleration,
              limits,
              dt);
      if (!retry.infeasible()) {
        omega = previous.omegaRadiansPerSecond;
        limited = retry;
      }
    }
    var velocity = limited.velocity();

    if (terminal.passType() == Goal.PassType.STOP) {
      boolean ready =
          !barrier.closed()
              && goalIndex >= activeGoals.size() - 1
              && path.length() - progress <= Math.max(CORRIDOR, tolerance.positionMeters() + 0.05)
              && goalDistance <= tolerance.positionMeters()
              && measured.getNorm() <= tolerance.linearSpeed()
              && velocity.getNorm() <= tolerance.linearSpeed()
              && Math.abs(targetHeading - heading.angle()) < Math.PI / 2
              && headingError <= tolerance.headingRadians()
              && Math.abs(measuredFieldSpeeds.omegaRadiansPerSecond) <= tolerance.angularSpeed()
              && Math.abs(omega) <= tolerance.angularSpeed()
              && !limited.infeasible();
      dwell = ready ? dwell + dt : 0;
      if (ready
          && dwell + 1e-9 >= tolerance.dwellSeconds()
          && oldVelocity.getNorm() <= Math.min(limits.braking(), traction) * dt + 1e-9
          && Math.abs(previous.omegaRadiansPerSecond) <= limits.maxAlpha() * dt + 1e-9) {
        if (terminal.marker() != null) {
          passed.add(terminal.marker());
        }
        status = RuntimeStatus.AT_GOAL;
        velocity = Translation2d.kZero;
        omega = 0;
      }
    }

    if (activeClearance.isPresent()
        && activeClearance.orElseThrow().clearance(measuredPose) < 0.0
        && diagnostic.isEmpty()) {
      diagnostic = "KEEPOUT_VIOLATION";
    }

    previous = new ChassisSpeeds(velocity.getX(), velocity.getY(), omega);
    boolean infeasible = limited.infeasible() || (beta < 1.0 - 1e-6 && profile.infeasible());
    LimitingFactor limiter =
        infeasible
            ? LimitingFactor.INFEASIBLE
            : (limited.limitingFactor() != LimitingFactor.NONE
                ? limited.limitingFactor()
                : profile.limitingFactor());
    result =
        new TickResult(previous, status, infeasible, limiter, generation, progress, diagnostic);
    return result;
  }

  /** Computes a drive command using the follower clock and field-relative measurements. */
  public TickResult update(Pose2d measuredPose, ChassisSpeeds measuredFieldSpeeds) {
    return update(clock.getAsDouble(), measuredPose, measuredFieldSpeeds);
  }

  private void activate(AutoSegment motionPlan, Optional<FieldClearance> clearance) {
    plan = requireNonNull(motionPlan);
    activeClearance = requireNonNull(clearance);
    limits = DrivetrainLimits.derive(robot, calibration, nominalDt, plan.limits());
    activeGoals = plan.goals();
    openedGates = new boolean[activeGoals.size()];
    passed.clear();
    headingArrivals.clear();
    path = null;
    remainingGoalStart = 0;
    goalIndex = 0;
    highWater = 0;
    dwell = 0;
    offTrack = 0;
    divergence = 0;
    lastTime = Double.NaN;
    relatchRequested = false;
    headingGoalIndex = -1;
    headingDwell = 0;
    gateWait = 0;
    turnHoldProgress = 0;
    turnHoldHeadingRadians = 0;
    terminalLatched = false;
    stopPositionLatched = false;
    stopHeadingLatched = false;
    rebuildStartPose = Pose2d.kZero;
    result =
        new TickResult(
            new ChassisSpeeds(),
            RuntimeStatus.RUNNING,
            false,
            LimitingFactor.NONE,
            generation,
            0,
            "");
  }

  private List<RotationBudgetEstimator.HeadingSpan> collectDownstreamHeadingSpans(
      double progress, double barrierProgress) {
    if (path == null || activeGoals.isEmpty()) {
      return ImmutableList.of();
    }
    List<RotationBudgetEstimator.HeadingSpan> spans = null;
    boolean appliedDelay = false;
    double spanStart = progress;
    double prevAngle = targetHeading;
    for (int i = goalIndex; i < activeGoals.size(); i++) {
      var goal = activeGoals.get(i);
      var keyframe = headingForGoal(goal);
      if (keyframe == null) {
        continue;
      }
      if (!appliedDelay) {
        appliedDelay = true;
        if (turnHoldProgress > spanStart && headingGoalIndex < 0) {
          spanStart = Math.min(barrierProgress, turnHoldProgress);
        }
      }
      double anchor =
          MathUtil.clamp(
              path.anchors().get(i) + keyframe.offsetMeters(), spanStart, barrierProgress);
      if (remainingGoalStart + i == headingGoalIndex) {
        spanStart = anchor;
        continue;
      }
      double targetAngle = prevAngle;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        targetAngle =
            prevAngle + turnDelta(prevAngle, fixed.angle().getRadians(), keyframe.direction());
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
      if (!HeadingSchedule.applies(goal) && Math.abs(delta) > 0.01) {
        if (spans == null) {
          spans = new ArrayList<>();
        }
        spans.add(new RotationBudgetEstimator.HeadingSpan(spanStart, anchor, delta));
      }
      prevAngle = targetAngle;
      spanStart = anchor;
    }
    return spans == null ? ImmutableList.of() : spans;
  }

  private double computeInitialTurnHold(Pose2d pose, PathBuilder.Path geometry, List<Goal> goals) {
    if (activeClearance.isEmpty()) {
      return 0.0;
    }
    var clearance = activeClearance.orElseThrow();
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      var keyframe = headingForGoal(goal);
      if (keyframe == null) {
        continue;
      }
      if (!(keyframe.target() instanceof HeadingPlan.Target.Fixed fixed)) {
        return 0.0;
      }
      double startHeading = pose.getRotation().getRadians();
      double diff = turnDelta(startHeading, fixed.angle().getRadians(), keyframe.direction());
      if (Math.abs(diff) <= 1e-4) {
        return 0.0;
      }
      double anchor =
          MathUtil.clamp(
              geometry.anchors().get(i) + keyframe.offsetMeters(), 0.0, geometry.length());
      int angleSteps = Math.max(1, (int) Math.ceil(Math.abs(diff) / Math.toRadians(10.0)));
      if (sweepClear(clearance, pose.getX(), pose.getY(), startHeading, diff, angleSteps)) {
        return 0.0;
      }
      for (double s = 0.025; s <= anchor + 1e-9; s += 0.025) {
        double sampleS = Math.min(s, anchor);
        geometry.sampleInto(sampleS, headingSampleScratch);
        if (sweepClear(
            clearance,
            headingSampleScratch[0],
            headingSampleScratch[1],
            startHeading,
            diff,
            angleSteps)) {
          return sampleS;
        }
      }
      return 0.0;
    }
    return 0.0;
  }

  private Barrier headingBarrier(
      Pose2d pose, ChassisSpeeds measured, PathBuilder.Projection projection, double dt) {
    double activation = 0;
    double terminal = path.anchors().get(activeGoals.size() - 1);
    targetDerivative = 0;
    faceTravel = false;
    if (turnHoldProgress > 0
        && highWater < turnHoldProgress - 1e-4
        && projection.progress() < turnHoldProgress - 1e-4) {
      targetHeading =
          heading.angle() + MathUtil.angleModulus(turnHoldHeadingRadians - heading.angle());
      headingGoalIndex = -1;
      headingDwell = 0;
      gateWait = 0;
      return nextClosedGate(0, 0, terminal);
    }
    for (int i = 0; i < activeGoals.size(); i++) {
      var goal = activeGoals.get(i);
      var keyframe = headingForGoal(goal);
      if (keyframe == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(path.anchors().get(i) + keyframe.offsetMeters(), activation, terminal);
      int globalIndex = remainingGoalStart + i;
      if (projection.progress() < activation) {
        break;
      }
      if (projection.progress() > anchor
          && i < activeGoals.size() - 1
          && (keyframe.gate() != HeadingPlan.GatePolicy.HARD || openedGates[globalIndex])) {
        activation = anchor;
        continue;
      }
      var sample = projection.sample();
      boolean entering = globalIndex != headingGoalIndex;
      double raw = entering ? pose.getRotation().getRadians() : targetHeading;
      if (keyframe.target() instanceof HeadingPlan.Target.Fixed fixed) {
        raw = fixed.angle().getRadians();
      } else if (keyframe.target() instanceof HeadingPlan.Target.Travel) {
        faceTravel = true;
        if (sample.tangent().getNorm() > 0) {
          raw = sample.tangent().getAngle().getRadians();
          targetDerivative = sample.curvature();
        }
      } else if (keyframe.target() instanceof HeadingPlan.Target.Point fixed) {
        var delta = fixed.target().minus(sample.position());
        double range = delta.getNorm();
        if (Double.isFinite(range) && range >= 0.25) {
          raw = delta.getAngle().getRadians() - fixed.mechanismOffset().getRadians();
          targetDerivative =
              MathUtil.clamp((range - 0.25) / 0.15, 0, 1)
                  * (-delta.getX() * sample.tangent().getY()
                      + delta.getY() * sample.tangent().getX())
                  / (range * range);
        }
      }
      if (entering) {
        targetHeading = heading.angle() + turnDelta(heading.angle(), raw, keyframe.direction());
        headingGoalIndex = globalIndex;
        headingDwell = 0;
        gateWait = 0;
      } else {
        targetHeading += MathUtil.angleModulus(raw - targetHeading);
      }
      if (keyframe.gate() == HeadingPlan.GatePolicy.HARD) {
        if (openedGates[globalIndex] && projection.progress() < anchor) {
          double dropoutError =
              Math.abs(MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()));
          if (dropoutError > 2.0 * keyframe.toleranceRadians()) {
            openedGates[globalIndex] = false;
            headingDwell = 0;
          }
        }
        if (!openedGates[globalIndex]) {
          double distance = pose.getTranslation().getDistance(path.sample(anchor).position());
          boolean ready =
              distance <= 0.25
                  && Math.abs(
                          MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()))
                      <= keyframe.toleranceRadians()
                  && Math.abs(measured.omegaRadiansPerSecond) <= keyframe.angularSpeedTolerance()
                  && Math.abs(targetHeading - heading.angle()) < Math.PI / 2;
          headingDwell = ready ? headingDwell + dt : 0;
          if (ready && headingDwell + 1e-9 >= keyframe.dwellSeconds()) {
            openedGates[globalIndex] = true;
          } else {
            gateWait = distance <= 0.25 ? gateWait + dt : 0;
            if ((projection.progress() > anchor && distance > 0.25) || gateWait > 3.0) {
              openedGates[globalIndex] = true;
              gateWait = -1.0;
            } else {
              return new Barrier(anchor, true);
            }
          }
        }
      }
      if (projection.progress() <= anchor || i == activeGoals.size() - 1) {
        return nextClosedGate(i + 1, anchor, terminal);
      }
      activation = anchor;
    }
    return new Barrier(path.length(), false);
  }

  private Barrier nextClosedGate(int from, double activation, double terminal) {
    for (int i = from; i < activeGoals.size(); i++) {
      var keyframe = activeGoals.get(i).heading();
      if (keyframe == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(path.anchors().get(i) + keyframe.offsetMeters(), activation, terminal);
      if (keyframe.gate() == HeadingPlan.GatePolicy.HARD && !openedGates[remainingGoalStart + i]) {
        return new Barrier(anchor, true);
      }
      activation = anchor;
    }
    return new Barrier(path.length(), false);
  }

  private VelocityProfiler.Result profileTranslation(
      double progress,
      double speed,
      double barrier,
      List<RotationBudgetEstimator.HeadingSpan> spans,
      boolean scheduled,
      double dt) {
    return VelocityProfiler.profile(
        path,
        activeGoals,
        progress,
        speed,
        barrier,
        (node, out) -> {
          RotationBudgetEstimator.estimateInto(
              limits,
              node,
              progress,
              faceTravel ? path.curvatureAt(node) : targetDerivative,
              scheduled ? 0 : heading.velocity(),
              scheduled ? 0 : referenceAlpha,
              scheduled ? 0 : targetHeading - heading.angle(),
              spans,
              out);
        },
        pathHeadings,
        limits,
        dt,
        profilerWorkspace);
  }

  private void rebuild(Pose2d pose, int from) {
    var candidate = plan.goals().subList(from, plan.goals().size());
    var geometry =
        PathBuilder.build(
            pose.getTranslation(),
            candidate,
            Math.min(limits.braking(), limits.friction()),
            activeClearance.orElse(null));
    checkArgument(geometry.length() <= 100, "Path exceeds profile capacity");
    remainingGoalStart = from;
    activeGoals = candidate;
    rebuildStartPose = pose;
    path = geometry;
    goalIndex = 0;
    generation++;
    highWater = 0;
    offTrack = 0;
    dwell = 0;
    terminalLatched = false;
    stopPositionLatched = false;
    stopHeadingLatched = false;
    turnHoldHeadingRadians = pose.getRotation().getRadians();
    turnHoldProgress = computeInitialTurnHold(pose, geometry, candidate);
    var passages = new ArrayList<HeadingPassage>();
    double precedingAnchor = 0;
    double terminal = path.anchors().get(activeGoals.size() - 1);
    for (int i = 0; i < candidate.size(); i++) {
      var goal = candidate.get(i);
      var keyframe = headingForGoal(goal);
      if (keyframe == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(
              path.anchors().get(i) + keyframe.offsetMeters(), precedingAnchor, terminal);
      if (HeadingSchedule.applies(goal)) {
        passages.add(new HeadingPassage(from + i, anchor, keyframe));
      }
      precedingAnchor = anchor;
    }
    headingPassages = passages;
    nextHeadingPassage = 0;
    previousHeadingProgress = 0;
    previousMeasuredHeading = pose.getRotation();
    pathHeadings =
        HeadingSchedule.build(
            path,
            candidate,
            turnHoldHeadingRadians,
            turnHoldProgress,
            activeClearance.orElse(null));
  }

  private boolean recordHeadingArrivals(double progress, Rotation2d measured, Barrier barrier) {
    boolean missed = false;
    while (nextHeadingPassage < headingPassages.size()) {
      var passage = headingPassages.get(nextHeadingPassage);
      if (progress + 1e-9 < passage.progress()
          || (barrier.closed() && barrier.progress() <= passage.progress() + 1e-9)) {
        break;
      }
      double advance = progress - previousHeadingProgress;
      var atAnchor =
          advance > 1e-9
              ? previousMeasuredHeading.interpolate(
                  measured, (passage.progress() - previousHeadingProgress) / advance)
              : measured;
      var fixed = (HeadingPlan.Target.Fixed) passage.keyframe().target();
      double error = fixed.angle().minus(atAnchor).getRadians();
      boolean within = Math.abs(error) <= passage.keyframe().toleranceRadians() + 1e-9;
      headingArrivals.add(new HeadingArrival(passage.goal(), error, within));
      missed |= !within;
      nextHeadingPassage++;
    }
    previousHeadingProgress = progress;
    previousMeasuredHeading = measured;
    return missed;
  }
}
