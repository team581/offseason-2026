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
import java.util.Collections;
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

  public static AutoSegment.Builder segment(String id) {
    return new AutoSegment.Builder(id);
  }

  private static double distanceToSegment(
      Translation2d point, Translation2d start, Translation2d end) {
    var delta = end.minus(start);
    double squared = delta.dot(delta);
    double fraction =
        squared > 1e-12 ? MathUtil.clamp(point.minus(start).dot(delta) / squared, 0, 1) : 0;
    return point.getDistance(start.plus(delta.times(fraction)));
  }

  private static boolean finite(ChassisSpeeds speed) {
    return Double.isFinite(speed.vxMetersPerSecond)
        && Double.isFinite(speed.vyMetersPerSecond)
        && Double.isFinite(speed.omegaRadiansPerSecond);
  }

  private static Translation2d translation(ChassisSpeeds speed) {
    return new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
  }

  private static boolean validPower(double value) {
    return Double.isFinite(value) && value > 0;
  }

  private final TimeSource clock;
  private final RobotDrivetrainSpec robot;
  private final EmpiricalDriveCalibration calibration;
  private final double nominalDt;
  private final Optional<FieldClearance> defaultFieldClearance;
  private Optional<FieldClearance> fieldClearance;
  private boolean marginWarning;
  private final Map<String, GoalPassageStatus> statuses = new LinkedHashMap<>();
  private final Set<Enum<?>> passed = new LinkedHashSet<>();
  private final Map<String, TimestampedTarget<Pose2d>> lastTargets = new HashMap<>();
  private final Map<String, Pose2d> geometryTargets = new HashMap<>();
  private final Set<String> openedGates = new LinkedHashSet<>();
  private AutoSegment plan;
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
  private boolean terminalLatched;

  private boolean stopping;

  private RuntimeStatus stopStatus;

  private String stopDiagnostic;

  private Translation2d lastPosition;

  private HeadingPlanner.State heading;

  private ChassisSpeeds previous;

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

  public boolean atGoal() {
    return result.status() == RuntimeStatus.AT_GOAL;
  }

  public void cancel() {
    pendingAssist = false;
    if (path != null && (result.status() == RuntimeStatus.RUNNING || atGoal())) {
      stop(RuntimeStatus.CANCELLED, "Cancelled");
    } else {
      result = zero(RuntimeStatus.CANCELLED, "Cancelled");
    }
  }

  public void driveTo(Pose2d pose, MotionLimits profile) {
    assistPose = pose;
    snapHeading = null;
    assistProfile = profile;
    pendingAssist = true;
  }

  public TickResult lastResult() {
    return result;
  }

  public Optional<MotionTelemetry> motionTelemetry() {
    return Optional.ofNullable(telemetry);
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
    var snapshot = input.robotState();
    if (!snapshot.enabled()) {
      pendingAssist = false;
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
      activationTime = now;
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
    double dt = Double.isNaN(lastTime) ? nominalDt : now - lastTime;
    if (!Double.isFinite(dt) || dt <= 0 || dt > 0.10 || !Double.isFinite(now)) {
      result = zero(RuntimeStatus.FAULT, "Invalid tick interval");
      return result;
    }
    lastTime = now;
    var pose = snapshot.measuredPose();
    var measuredSpeeds = snapshot.measuredFieldSpeeds();
    if (!snapshot.localizationValid()
        || !Double.isFinite(snapshot.poseTimestampSeconds())
        || snapshot.poseTimestampSeconds() > now + 1e-6
        || now - snapshot.poseTimestampSeconds() > 0.10
        || !Double.isFinite(pose.getTranslation().getNorm())
        || !Double.isFinite(pose.getRotation().getRadians())
        || !finite(measuredSpeeds)
        || snapshot.appliedFieldSpeeds().filter(speed -> !finite(speed)).isPresent()
        || !validPower(snapshot.swerveSupplyCurrentLimitAmps().orElse(1))
        || !validPower(snapshot.restingBatteryVoltageVolts().orElse(12))) {
      result = zero(RuntimeStatus.FAULT, "Invalid or stale robot snapshot");
      return result;
    }
    var measured = translation(measuredSpeeds);
    try {
      if (path == null) {
        previous = snapshot.appliedFieldSpeeds().orElse(measuredSpeeds);
        heading =
            new HeadingPlanner.State(
                pose.getRotation().getRadians(), previous.omegaRadiansPerSecond);
        headingId = null;
        referenceAlpha = 0;
        targetHeading = heading.angle();
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
      String diagnostic = marginWarning ? "KEEPOUT_MARGIN_WARNING" : "";
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
      offTrack =
          snapHeading == null
                  && (projected.distance() > CORRIDOR || highWater - projected.progress() > 0.55)
              ? offTrack + dt
              : 0;
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
      var newMarkers = new LinkedHashSet<Enum<?>>();
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
        recordPass(goal, closest <= goal.withinMeters(), newMarkers);
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
      var profile =
          VelocityProfiler.profile(
              path,
              horizon,
              plan.zones(),
              progress,
              along,
              barrier.progress(),
              node ->
                  RotationBudgetEstimator.estimate(
                      limits,
                      faceTravel ? path.sample(node).curvature() : targetDerivative,
                      node - progress,
                      heading.velocity(),
                      referenceAlpha,
                      targetHeading - heading.angle()),
              limits,
              motor,
              dt);
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
      var turn =
          HeadingPlanner.step(
              heading,
              targetHeading,
              targetDerivative,
              pose.getRotation().getRadians(),
              previous.omegaRadiansPerSecond,
              headingSpeed,
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
      if (limited.infeasible() && omega != previous.omegaRadiansPerSecond) {
        var retry =
            VectorLimiter.project(
                demand,
                oldVelocity,
                pose.getRotation(),
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
          recordPass(terminal, true, newMarkers);
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
              limited.infeasible()
                  ? LimitingFactor.INFEASIBLE
                  : (velocity.getDistance(demand) > 1e-6
                      ? LimitingFactor.VECTOR_SLEW_LIMIT
                      : LimitingFactor.NONE),
              Collections.unmodifiableSet(newMarkers),
              Map.copyOf(statuses),
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
    fieldClearance = clearance;
    assist = false;
    pendingAssist = false;
    snapHeading = null;
    lastTargets.clear();
    openedGates.clear();
    headingId = null;
    highWater = 0;
    horizonStart = index;
    statuses.clear();
    passed.clear();
    for (var goal : motionPlan.goals().subList(0, index)) {
      if (goal.marker() != null) {
        passed.add(goal.marker());
      }
    }
    limits = DrivetrainLimits.derive(robot, calibration, nominalDt, plan.limits());
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
              Map.copyOf(statuses),
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
    double smallest = Double.POSITIVE_INFINITY;
    int checks = Math.max(1, (int) Math.ceil(pose.getTranslation().getDistance(arrest) / 0.015));
    for (int i = 0; i <= checks; i++) {
      var position = pose.getTranslation().interpolate(arrest, (double) i / checks);
      smallest = Math.min(smallest, clearance.clearance(new Pose2d(position, pose.getRotation())));
    }
    int intervals = Math.max(1, (int) Math.ceil(geometry.length() / 0.015));
    var finish = goals.get(goals.size() - 1).pose().getRotation();
    for (int i = 0; i <= intervals; i++) {
      var position = geometry.sample(geometry.length() * i / intervals).position();
      for (int angleStep = 0; angleStep <= 36; angleStep++) {
        var rotation = pose.getRotation().interpolate(finish, angleStep / 36.0);
        smallest = Math.min(smallest, clearance.clearance(new Pose2d(position, rotation)));
      }
    }
    if (smallest <= 0 || (!recovery && smallest < clearance.margin())) {
      throw new IllegalArgumentException("Path footprint intersects a keepout or required margin");
    }
    marginWarning = smallest < clearance.margin();
    if (marginWarning) {
      limits = DrivetrainLimits.derive(robot, calibration, nominalDt, plan.limits(), 0.60);
    }
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
    for (int i = 0; i < horizon.size(); i++) {
      var goal = resolvedGoals.get(horizonStart + i);
      var keyframe = goal.heading();
      if (keyframe == null && goal.passType() == Goal.PassType.STOP) {
        keyframe = HeadingPlan.to(goal.pose().getRotation());
      }
      if (keyframe == null) {
        continue;
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
      if (keyframe.gate() == HeadingPlan.GatePolicy.HARD && !openedGates.contains(goal.id())) {
        double distance = pose.getTranslation().getDistance(path.sample(anchor).position());
        boolean ready =
            !singularAim
                && distance <= 0.25
                && Math.abs(MathUtil.angleModulus(targetHeading - pose.getRotation().getRadians()))
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
      if (projection.progress() <= anchor || i == horizon.size() - 1) {
        return nextClosedGate(i + 1, anchor, terminal);
      }
      activation = anchor;
    }
    return new Barrier(path.length(), false);
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
    var candidate = resolvedGoals.subList(from, end + 1);
    var geometry =
        PathBuilder.build(
            pose.getTranslation(), candidate, Math.min(limits.braking(), limits.friction()));
    if (geometry.length() > 100) {
      throw new IllegalArgumentException("Path exceeds profile capacity");
    }
    checkClearance(pose, geometry, candidate, path != null && !assist);
    horizonStart = from;
    horizon = List.copyOf(candidate);
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

  private void recordPass(Goal goal, boolean within, Set<Enum<?>> newMarkers) {
    statuses.put(
        goal.id(),
        within
            ? GoalPassageStatus.PASSED_WITHIN_TOLERANCE
            : GoalPassageStatus.PASSED_OUTSIDE_TOLERANCE);
    if (goal.marker() != null && passed.add(goal.marker())) {
      newMarkers.add(goal.marker());
    }
  }

  private boolean resolveGoals(TickInput input, int from) {
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
    marginWarning = false;
    lastTime = Double.NaN;
    activationTime = clock.getTimestampSeconds();
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
    double squaredRadii = robot.modulePositionsMeters().stream().mapToDouble(p -> p.dot(p)).sum();
    double torque = robot.yawMoiKgMetersSq() * alpha / squaredRadii;
    double cap = robot.massKg() * limits.friction() / 4;
    return robot.modulePositionsMeters().stream()
        .map(location -> {
          var r = location.rotateBy(pose.getRotation());
          var force =
              acceleration
                  .times(robot.massKg() / 4)
                  .plus(new Translation2d(-r.getY(), r.getX()).times(torque));
          return force.getNorm() > cap ? force.times(cap / force.getNorm()) : force;
        })
        .toList();
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
        Map.copyOf(statuses),
        generation,
        highWater,
        diagnostic);
  }
}
