package frc.robot.autos;

import static java.util.Objects.requireNonNull;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.trailblazer3.runtime.Trailblazer.PreparedSegment;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import frc.robot.robot_manager.RobotManager;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;

/** Auto state machine with alliance conversion and asynchronous path preparation. */
public abstract class TrailblazerAuto<S extends Enum<S>> extends StateMachine<S> {
  private enum Drive implements DriveState {
    IDLE,
    FAILED
  }

  private sealed interface DriveState {}

  private record Failed(String reason) implements Plan {}

  private sealed interface Plan {}

  private record Planning(CompletableFuture<Plan> result, boolean live) {}

  private record Ready(PreparedSegment segment, ChassisSpeeds launch) implements Plan {}

  private record Waiting(AutoSegment segment) implements DriveState {}

  private static final double ADVISORY_START_POS_TOLERANCE_METERS = 0.25;
  private static final double ADVISORY_START_HEADING_TOLERANCE_DEGREES = 15.0;

  private static void logAuthoredWaypoints(AutoSegment segment) {
    DogLog.log(
        "Trailblazer3/AuthoredWaypoints/" + segment.id(),
        segment.goals().stream().map(goal -> goal.pose()).toArray(Pose2d[]::new));
  }

  protected final RobotManager robot;
  protected final Trailblazer trailblazer;
  protected final Alliance alliance;
  private final Pose2d allianceStartPose;
  private final List<AutoSegment> authoredSegments;
  private final Executor pathExecutor;
  private final Map<AutoSegment, Planning> paths = new HashMap<>();
  private DriveState driveState = Drive.IDLE;
  private boolean preparationQueued;
  private boolean startedRunning;
  private S trackedState;
  private double stateStartSeconds;

  protected TrailblazerAuto(
      S initialState, Pose2d blueStartPose, AutoContext context, AutoSegment... blueSegments) {
    super(requireNonNull(initialState));
    requireNonNull(context);
    this.trackedState = initialState;
    this.robot = requireNonNull(context.robot());
    this.trailblazer = requireNonNull(context.trailblazer());
    this.alliance = requireNonNull(context.alliance());
    this.allianceStartPose = fieldPose(requireNonNull(blueStartPose));
    this.authoredSegments = List.of(blueSegments);
    this.pathExecutor = requireNonNull(context.pathExecutor());
  }

  /** Returns the current state name for dashboard telemetry. */
  public final String currentStepName() {
    return getState().name();
  }

  /** Returns whether the enabled loop has started executing on this routine. */
  public final boolean hasStartedRunning() {
    return startedRunning;
  }

  /** Advisory check used for dashboard diagnostics before enabling; never aborts autonomous. */
  public final boolean isNearStartPose(Pose2d actualPose) {
    var expected = allianceStartPose;
    return actualPose.getTranslation().getDistance(expected.getTranslation())
            <= ADVISORY_START_POS_TOLERANCE_METERS
        && MathUtil.isNear(
            0,
            actualPose.getRotation().minus(expected.getRotation()).getDegrees(),
            ADVISORY_START_HEADING_TOLERANCE_DEGREES);
  }

  @Override
  public final void periodic() {
    if (!startedRunning) {
      startedRunning = true;
      trackedState = getState();
      stateStartSeconds = trailblazer.nowSeconds();
    }
    if (driveState instanceof Waiting waiting) {
      continueDrive(waiting.segment(), robot.localization.getPose());
      if (driveState == Drive.FAILED) {
        robot.idleRequest();
      }
    }
    if (driveState != Drive.IDLE) {
      return;
    }
    double now = trailblazer.nowSeconds();
    collectInputs();
    super.periodic();
    if (driveState == Drive.FAILED) {
      robot.idleRequest();
    }
    if (getState() != trackedState) {
      trackedState = getState();
      stateStartSeconds = now;
    }
  }

  /** Returns the alliance-adjusted expected starting pose. */
  public final Pose2d startPose() {
    return allianceStartPose;
  }

  private void continueDrive(AutoSegment segment, Pose2d pose) {
    requestPath(segment, pose);
    var completed = paths.get(segment).result().getNow(null);
    if (completed instanceof Ready ready && ready.segment().acceptsStart(pose)) {
      if (!trailblazer.isStaged(ready.segment())) {
        trailblazer.activate(ready.segment());
      }
      driveState = Drive.IDLE;
    } else if (completed instanceof Failed failed) {
      trailblazer.cancel();
      driveState = Drive.FAILED;
      DogLog.log("Trailblazer3/PathPreparationFailure", failed.reason());
    } else {
      trailblazer.cancel();
      DogLog.log("Trailblazer3/WaitingForPath", segment.id());
      return;
    }
    DogLog.log("Trailblazer3/WaitingForPath", "");
  }

  // TODO: remove if we decide against replanning paths if pose is off
  private Plan preparePath(AutoSegment blueSegment, AutoSegment fieldSegment, Pose2d start) {
    try {
      var plan = trailblazer.prepare(fieldSegment, CompAutoField.clearance(alliance), start);
      var speeds = new ChassisSpeeds();
      if (blueSegment.equals(authoredSegments.get(0))) {
        var stagingFollower = CompTrailblazer.create();
        stagingFollower.activate(plan);
        var result = stagingFollower.update(0.0, start, speeds);
        if (result.status() == RuntimeStatus.RUNNING
            && result.diagnostic().isEmpty()
            && !result.infeasible()) {
          speeds = result.commandedFieldSpeeds();
        }
      }
      return new Ready(plan, speeds);
    } catch (RuntimeException exception) {
      var reason = fieldSegment.id() + ": " + exception;
      DriverStation.reportWarning("Trailblazer3 " + reason, false);
      return new Failed(reason);
    }
  }

  private void queuePreparation(Pose2d start) {
    var nextStart = CompletableFuture.completedFuture(start);
    for (var blueSegment : authoredSegments) {
      var fieldSegment = CompAutoField.segment(blueSegment, alliance);
      var result =
          nextStart.thenApplyAsync(
              pose -> preparePath(blueSegment, fieldSegment, pose), pathExecutor);
      paths.put(blueSegment, new Planning(result, false));
      nextStart =
          result.thenApply(plan ->
              plan instanceof Ready ready
                  ? ready.segment().endPose()
                  : fieldSegment.goals().get(fieldSegment.goals().size() - 1).pose());
    }
  }

  private void requestPath(AutoSegment segment, Pose2d pose) {
    if (paths.containsKey(segment)) {
      var planning = paths.get(segment);
      if (!planning.result().isDone()) {
        return;
      }
      var plan = planning.result().join();
      if ((plan instanceof Ready ready && ready.segment().acceptsStart(pose))
          || (plan instanceof Failed && planning.live())) {
        return;
      }
    }
    var fieldSegment = CompAutoField.segment(segment, alliance);
    paths.put(
        segment,
        new Planning(
            CompletableFuture.supplyAsync(
                () -> preparePath(segment, fieldSegment, pose), pathExecutor),
            true));
  }

  /** Returns whether Trailblazer has completed the currently active segment. */
  protected final boolean atGoal() {
    return trailblazer.atGoal();
  }

  /** Activates a Blue-alliance {@link AutoSegment} with default alliance bumper clearance. */
  protected final void drive(AutoSegment blueSegment) {
    var segment = CompAutoField.segment(requireNonNull(blueSegment), alliance);
    driveState = Drive.IDLE;
    if (authoredSegments.contains(blueSegment)) {
      driveState = new Waiting(blueSegment);
      if (startedRunning) {
        continueDrive(blueSegment, robot.localization.getPose());
      }
    } else {
      trailblazer.activate(segment, CompAutoField.clearance(alliance));
    }
    logAuthoredWaypoints(segment);
  }

  /** Activates a Blue-alliance {@link AutoSegment} without keepout clearance checking. */
  protected final void driveWithoutClearance(AutoSegment blueSegment) {
    driveState = Drive.IDLE;
    var segment = CompAutoField.segment(requireNonNull(blueSegment), alliance);
    trailblazer.activateWithoutClearance(segment);
    logAuthoredWaypoints(segment);
  }

  protected final Pose2d fieldPose(Pose2d bluePose) {
    return CompAutoField.pose(bluePose, alliance);
  }

  /** Returns whether the robot has passed {@code marker} in the active segment. */
  protected final boolean passedMarker(Enum<?> marker) {
    return trailblazer.passedMarker(requireNonNull(marker));
  }

  /** Refreshes an upcoming segment from the current pose while the auto is scoring. */
  protected final void prepareNext(AutoSegment blueSegment) {
    requestPath(blueSegment, robot.localization.getPose());
  }

  /** Stops the path follower and clears the active segment (for stationary shooting/waiting). */
  protected final void stopDriving() {
    driveState = Drive.IDLE;
    trailblazer.cancel();
  }

  @Override
  protected final boolean timeout(double durationSeconds) {
    double now = trailblazer.nowSeconds();
    if (getState() != trackedState) {
      trackedState = getState();
      stateStartSeconds = now;
    }
    return (now - stateStartSeconds) > durationSeconds;
  }

  final boolean canDrive() {
    return driveState == Drive.IDLE;
  }

  final void discardPreparation() {
    paths.values().forEach(path -> path.result().cancel(false));
  }

  final boolean hasQueuedPreparation() {
    return preparationQueued;
  }

  final void prepareWhileDisabled(Pose2d pose, boolean autonomous, boolean stageLaunch) {
    if (!preparationQueued) {
      queuePreparation(stageLaunch && isNearStartPose(pose) ? pose : allianceStartPose);
      preparationQueued = true;
    }
    if (!autonomous) {
      return;
    }
    initializeState();
    if (!(driveState instanceof Waiting waiting)) {
      return;
    }
    var segment = waiting.segment();
    if (stageLaunch && isNearStartPose(pose)) {
      requestPath(segment, pose);
    }
    if (paths.get(segment).result().getNow(null) instanceof Ready ready
        && !trailblazer.isStaged(ready.segment())) {
      // Install geometry for telemetry while the live follower and auto state machine are paused.
      trailblazer.activate(ready.segment());
    }
  }

  final ChassisSpeeds stagedLaunch(Pose2d pose) {
    if (driveState instanceof Waiting waiting
        && isNearStartPose(pose)
        && paths.get(waiting.segment()).result().getNow(null) instanceof Ready ready
        && ready.segment().acceptsStart(pose)) {
      return ready.launch();
    }
    return new ChassisSpeeds();
  }
}
