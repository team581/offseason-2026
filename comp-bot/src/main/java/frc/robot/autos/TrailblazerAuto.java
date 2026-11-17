package frc.robot.autos;

import static java.util.Objects.requireNonNull;

import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.robot_manager.RobotManager;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * State-machine base class for Trailblazer 3 autonomous routines.
 *
 * <p>Each auto defines an {@code enum State} and implements standard Team 581 {@link StateMachine}
 * hooks ({@link #afterTransition}, {@link #getNextState}, and {@link #whileInState}). All
 * coordinates are authored in the canonical Blue-alliance frame and automatically flipped for the
 * active alliance.
 */
public abstract class TrailblazerAuto<S extends Enum<S>> extends StateMachine<S> {
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
  private final Optional<Pose2d> allianceStartPose;

  private final List<AutoSegment> alliancePreviewSegments;
  private boolean startedRunning = false;
  private @Nullable S trackedState;

  private double stateStartSeconds = Double.NaN;

  /** Creates an auto without a fixed field starting pose (such as {@link DoNothingAuto}). */
  protected TrailblazerAuto(S initialState, AutoContext context) {
    this(initialState, null, context);
  }

  /** Creates an auto with a Blue-alliance nominal start pose and ordered preview segments. */
  protected TrailblazerAuto(
      S initialState,
      @Nullable Pose2d blueStartPose,
      AutoContext context,
      AutoSegment... bluePreviewSegments) {
    super(requireNonNull(initialState));
    requireNonNull(context);
    this.robot = requireNonNull(context.robot());
    this.trailblazer = requireNonNull(context.trailblazer());
    this.alliance = requireNonNull(context.alliance());
    this.allianceStartPose =
        blueStartPose == null
            ? Optional.empty()
            : Optional.of(CompAutoField.pose(blueStartPose, alliance));
    this.alliancePreviewSegments =
        Arrays.stream(bluePreviewSegments)
            .map(segment -> CompAutoField.segment(requireNonNull(segment), alliance))
            .toList();
  }

  /** Returns the current state name for dashboard telemetry. */
  public final String currentStepName() {
    return getState().name();
  }

  /** Returns whether {@link #periodic} has started executing on this routine. */
  public final boolean hasStartedRunning() {
    return startedRunning;
  }

  /** Computes a unit-speed field-relative velocity for disabled swerve module pre-alignment. */
  public final Optional<ChassisSpeeds> initialDisabledWheelTarget(Pose2d currentPose) {
    if (alliancePreviewSegments.isEmpty()) {
      return Optional.empty();
    }
    var target = alliancePreviewSegments.get(0).goals().get(0).pose().getTranslation();
    var origin = allianceStartPose.orElse(currentPose).getTranslation();
    var delta = target.minus(origin);
    double norm = delta.getNorm();
    return norm > 1e-4
        ? Optional.of(new ChassisSpeeds(delta.getX() / norm, delta.getY() / norm, 0.0))
        : Optional.empty();
  }

  /** Advisory check used for dashboard diagnostics before enabling; never aborts autonomous. */
  public final boolean isNearStartPose(Pose2d actualPose) {
    if (allianceStartPose.isEmpty()) {
      return true;
    }
    var expected = allianceStartPose.orElseThrow();
    return actualPose.getTranslation().getDistance(expected.getTranslation())
            <= ADVISORY_START_POS_TOLERANCE_METERS
        && MathUtil.isNear(
            0,
            actualPose.getRotation().minus(expected.getRotation()).getDegrees(),
            ADVISORY_START_HEADING_TOLERANCE_DEGREES);
  }

  @Override
  public final void periodic() {
    double now = trailblazer.nowSeconds();
    if (!startedRunning) {
      startedRunning = true;
      trackedState = getState();
      stateStartSeconds = now;
    }
    collectInputs();
    super.periodic();
    if (getState() != trackedState) {
      trackedState = getState();
      stateStartSeconds = now;
    }
  }

  /** Samples the stitched preview polyline across all preview segments for AdvantageScope. */
  public final Pose2d[] previewPath(Pose2d fallbackStartPose) {
    Pose2d cursor = allianceStartPose.orElse(fallbackStartPose);
    var clearance = CompAutoField.clearance(alliance);
    var poses = new ArrayList<Pose2d>();
    for (var segment : alliancePreviewSegments) {
      var goals = segment.goals();
      try {
        var geometry = PathBuilder.build(cursor.getTranslation(), goals, 1.6, clearance);
        var sampled = Trailblazer.sampleSegmentPoses(cursor, geometry, goals, 0.10);
        for (int i = poses.isEmpty() ? 0 : 1; i < sampled.length; i++) {
          poses.add(sampled[i]);
        }
        cursor =
            sampled.length > 0 ? sampled[sampled.length - 1] : goals.get(goals.size() - 1).pose();
      } catch (IllegalArgumentException ignored) {
        if (poses.isEmpty()) {
          poses.add(cursor);
        }
        for (var goal : goals) {
          poses.add(goal.pose());
        }
        cursor = goals.get(goals.size() - 1).pose();
      }
    }
    if (poses.isEmpty() && allianceStartPose.isPresent()) {
      poses.add(allianceStartPose.orElseThrow());
    }
    return poses.toArray(Pose2d[]::new);
  }

  /** Returns the alliance-adjusted expected starting pose, if this auto specifies one. */
  public final Optional<Pose2d> startPose() {
    return allianceStartPose;
  }

  /** Returns whether Trailblazer has completed the currently active segment. */
  protected final boolean atGoal() {
    return trailblazer.atGoal();
  }

  /** Activates a Blue-alliance {@link AutoSegment} with default alliance bumper clearance. */
  protected final void drive(AutoSegment blueSegment) {
    var segment = CompAutoField.segment(requireNonNull(blueSegment), alliance);
    trailblazer.activate(segment, CompAutoField.clearance(alliance));
    logAuthoredWaypoints(segment);
  }

  /** Activates a Blue-alliance {@link AutoSegment} without keepout clearance checking. */
  protected final void driveWithoutClearance(AutoSegment blueSegment) {
    var segment = CompAutoField.segment(requireNonNull(blueSegment), alliance);
    trailblazer.activateWithoutClearance(segment);
    logAuthoredWaypoints(segment);
  }

  /** Returns whether the robot has passed {@code marker} in the active segment. */
  protected final boolean passedMarker(Enum<?> marker) {
    return trailblazer.passedMarker(requireNonNull(marker));
  }

  /** Stops the path follower and clears the active segment (for stationary shooting/waiting). */
  protected final void stopDriving() {
    trailblazer.cancel();
  }

  @Override
  protected final boolean timeout(double durationSeconds) {
    double now = trailblazer.nowSeconds();
    if (getState() != trackedState) {
      trackedState = getState();
      stateStartSeconds = now;
    }
    return (now - stateStartSeconds) >= durationSeconds;
  }
}
