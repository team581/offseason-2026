package frc.robot.autos;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkState;
import static java.util.Objects.requireNonNull;

import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.robot_manager.RobotManager;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;

/**
 * Student-friendly base class for Trailblazer 3 autonomous routines.
 *
 * <p>Authors write steps top-to-bottom in the constructor using {@link #action}, {@link #drive},
 * {@link #waitFor}, {@link #waitSeconds}, and {@link #branch}. All poses and paths are authored in
 * canonical Blue-alliance coordinates and automatically flipped for the active alliance.
 */
public abstract class TrailblazerAuto {
  /** Fluent configuration handle for a driving step. */
  public final class DriveStep {
    private final String name;
    private final AutoSegment allianceSegment;
    private @Nullable FieldClearance allianceClearance;
    private boolean disableClearance = false;
    private Runnable onStartAction;
    private Runnable everyLoopAction;
    private final List<MarkerTrigger> markerTriggers = new ArrayList<>();
    private BooleanSupplier releaseCondition;
    private BooleanSupplier earlyExitCondition;
    private double timeoutSeconds = Double.POSITIVE_INFINITY;
    private double startTimeSeconds;

    private DriveStep(String name, AutoSegment blueSegment) {
      this.name = validateStepName(name);
      this.allianceSegment = CompAutoField.segment(requireNonNull(blueSegment), alliance);
    }

    /** Runs {@code action} every loop while the robot is driving this step. */
    public DriveStep everyLoop(Runnable action) {
      this.everyLoopAction = requireNonNull(action);
      return this;
    }

    /** Fires {@code action} once when the robot passes {@code marker} along this segment. */
    public DriveStep onMarker(Enum<?> marker, Runnable action) {
      markerTriggers.add(new MarkerTrigger(marker, action));
      return this;
    }

    /** Runs {@code action} once when this driving step starts. */
    public DriveStep onStart(Runnable action) {
      this.onStartAction = requireNonNull(action);
      return this;
    }

    /** Releases a paused intermediate stop inside this segment once {@code ready} is true. */
    public DriveStep releaseWhen(BooleanSupplier ready) {
      this.releaseCondition = requireNonNull(ready);
      return this;
    }

    /** Finishes this drive step early and moves to the next step if {@code condition} is true. */
    public DriveStep until(BooleanSupplier condition) {
      this.earlyExitCondition = requireNonNull(condition);
      return this;
    }

    /** Checks robot bumpers against a Blue-origin {@link FieldClearance} keepout map. */
    public DriveStep withClearance(FieldClearance blueClearance) {
      this.disableClearance = false;
      this.allianceClearance = requireNonNull(blueClearance).forAlliance(alliance);
      return this;
    }

    /** Disables keepout clearance checking for this driving step. */
    public DriveStep withoutClearance() {
      this.disableClearance = true;
      this.allianceClearance = null;
      return this;
    }

    /** Caps how long this drive step is allowed to run before advancing to the next step. */
    public DriveStep withTimeout(double maxSeconds) {
      checkArgument(Double.isFinite(maxSeconds) && maxSeconds > 0, "Step timeout must be positive");
      this.timeoutSeconds = maxSeconds;
      return this;
    }

    private void activate() {
      if (disableClearance) {
        trailblazer.activateWithoutClearance(allianceSegment);
      } else if (allianceClearance != null) {
        trailblazer.activate(allianceSegment, allianceClearance);
      } else {
        trailblazer.activate(allianceSegment);
      }
    }

    private Step asStep() {
      return new Step() {
        @Override
        public void enter(double nowSeconds) {
          startTimeSeconds = nowSeconds;
          for (var trigger : markerTriggers) {
            trigger.fired = false;
          }
          activate();
          if (onStartAction != null) {
            onStartAction.run();
          }
        }

        @Override
        public void flushMarkers() {
          for (var trigger : markerTriggers) {
            if (!trigger.fired && trailblazer.passedMarker(trigger.marker)) {
              trigger.fired = true;
              trigger.action.run();
            }
          }
        }

        @Override
        public boolean isComplete(double nowSeconds) {
          if (trailblazer.atGoal()) {
            if (releaseCondition == null) {
              return true;
            }
            if (releaseCondition.getAsBoolean() && trailblazer.release()) {
              return false;
            }
            if (releaseCondition.getAsBoolean()) {
              return true;
            }
          }
          if (earlyExitCondition != null && earlyExitCondition.getAsBoolean()) {
            trailblazer.cancel();
            return true;
          }
          if (Double.isFinite(timeoutSeconds)
              && (nowSeconds - startTimeSeconds) >= timeoutSeconds) {
            trailblazer.cancel();
            return true;
          }
          return false;
        }

        @Override
        public String name() {
          return name;
        }

        @Override
        public void periodic() {
          if (everyLoopAction != null) {
            everyLoopAction.run();
          }
        }
      };
    }
  }

  /** Fluent configuration handle for a waiting step. */
  public final class WaitStep {
    private final String name;
    private final double timeoutSeconds;
    private final BooleanSupplier condition;
    private Runnable onStartAction;
    private Runnable everyLoopAction;
    private double startTimeSeconds;

    private WaitStep(String name, double timeoutSeconds, BooleanSupplier condition) {
      this.name = validateStepName(name);
      checkArgument(
          Double.isFinite(timeoutSeconds) && timeoutSeconds >= 0,
          "Wait timeout must be nonnegative");
      this.timeoutSeconds = timeoutSeconds;
      this.condition = requireNonNull(condition);
    }

    /** Runs {@code action} every loop while waiting. */
    public WaitStep everyLoop(Runnable action) {
      this.everyLoopAction = requireNonNull(action);
      return this;
    }

    /** Runs {@code action} once when this wait step begins. */
    public WaitStep onStart(Runnable action) {
      this.onStartAction = requireNonNull(action);
      return this;
    }

    private Step asStep() {
      return new Step() {
        @Override
        public void enter(double nowSeconds) {
          startTimeSeconds = nowSeconds;
          trailblazer.cancel();
          if (onStartAction != null) {
            onStartAction.run();
          }
        }

        @Override
        public boolean isComplete(double nowSeconds) {
          return condition.getAsBoolean() || (nowSeconds - startTimeSeconds) >= timeoutSeconds;
        }

        @Override
        public String name() {
          return name;
        }

        @Override
        public void periodic() {
          if (everyLoopAction != null) {
            everyLoopAction.run();
          }
        }
      };
    }
  }

  private record ActionStep(String name, Runnable action) implements Step {
    private ActionStep {
      name = validateStepName(name);
      requireNonNull(action);
    }

    @Override
    public void enter(double nowSeconds) {
      action.run();
    }

    @Override
    public boolean isComplete(double nowSeconds) {
      return true;
    }

    @Override
    public boolean isInstant() {
      return true;
    }
  }

  private final class BranchStep implements Step {
    private final String name;
    private final BooleanSupplier condition;
    private final Runnable ifTrue;
    private final Runnable ifFalse;

    private BranchStep(String name, BooleanSupplier condition, Runnable ifTrue, Runnable ifFalse) {
      this.name = validateStepName(name);
      this.condition = requireNonNull(condition);
      this.ifTrue = requireNonNull(ifTrue);
      this.ifFalse = requireNonNull(ifFalse);
    }

    @Override
    public void enter(double nowSeconds) {
      var chosen = condition.getAsBoolean() ? ifTrue : ifFalse;
      var collected = new ArrayList<Step>();
      var previousTarget = pendingBranchSteps;
      pendingBranchSteps = collected;
      try {
        chosen.run();
      } finally {
        pendingBranchSteps = previousTarget;
      }
      if (!collected.isEmpty()) {
        steps.addAll(stepIndex + 1, collected);
      }
    }

    @Override
    public boolean isComplete(double nowSeconds) {
      return true;
    }

    @Override
    public boolean isInstant() {
      return true;
    }

    @Override
    public String name() {
      return name;
    }
  }

  private static final class MarkerTrigger {
    final Enum<?> marker;
    final Runnable action;
    boolean fired;

    MarkerTrigger(Enum<?> marker, Runnable action) {
      this.marker = requireNonNull(marker);
      this.action = requireNonNull(action);
    }
  }

  private interface Step {
    void enter(double nowSeconds);

    default void flushMarkers() {}

    boolean isComplete(double nowSeconds);

    default boolean isInstant() {
      return false;
    }

    String name();

    default void periodic() {}
  }

  private static final double ADVISORY_START_POS_TOLERANCE_METERS = 0.25;

  private static final double ADVISORY_START_HEADING_TOLERANCE_DEGREES = 15.0;

  private static String validateStepName(String name) {
    checkArgument(name != null && !name.isBlank(), "Auto step name cannot be blank");
    return name;
  }

  protected final RobotManager robot;
  protected final Trailblazer trailblazer;
  protected final Alliance alliance;
  private final Optional<Pose2d> allianceStartPose;
  private final List<Step> steps = new ArrayList<>();
  private final List<DriveStep> driveSteps = new ArrayList<>();
  private @Nullable DriveStep firstDriveStep;
  private int stepIndex = 0;
  private boolean stepEntered = false;
  private boolean startedRunning = false;
  private @Nullable List<Step> pendingBranchSteps = null;

  /** Creates an auto without a fixed field starting pose (such as {@link DoNothingAuto}). */
  protected TrailblazerAuto(AutoContext context) {
    this((Pose2d) null, context);
  }

  /** Creates an auto with a nominal starting pose in canonical Blue-alliance field coordinates. */
  protected TrailblazerAuto(Pose2d blueStartPose, AutoContext context) {
    requireNonNull(context);
    this.robot = requireNonNull(context.robot());
    this.trailblazer = requireNonNull(context.trailblazer());
    this.alliance = requireNonNull(context.alliance());
    this.allianceStartPose =
        blueStartPose == null
            ? Optional.empty()
            : Optional.of(CompAutoField.pose(blueStartPose, alliance));
  }

  /**
   * Package-private constructor for pure step-sequencing unit tests without a {@link RobotManager}.
   */
  TrailblazerAuto(Pose2d blueStartPose, Trailblazer trailblazer, Alliance alliance) {
    this.robot = null;
    this.trailblazer = requireNonNull(trailblazer);
    this.alliance = requireNonNull(alliance);
    this.allianceStartPose =
        blueStartPose == null
            ? Optional.empty()
            : Optional.of(CompAutoField.pose(blueStartPose, alliance));
  }

  /** Returns the current step index (or {@code steps.size()} when done). */
  public final int currentStepIndex() {
    return stepIndex;
  }

  /** Returns the human-readable name of the current step, or {@code "DONE"}. */
  public final String currentStepName() {
    return isFinished() ? "DONE" : steps.get(stepIndex).name();
  }

  /** Returns whether {@link #periodic} has started executing steps on this routine. */
  public final boolean hasStartedRunning() {
    return startedRunning;
  }

  /** Calculates the first drive demand without entering any auto steps or firing callbacks. */
  public final Optional<ChassisSpeeds> initialDisabledWheelTarget(Pose2d currentPose) {
    if (firstDriveStep == null) {
      return Optional.empty();
    }
    firstDriveStep.activate();
    double now = trailblazer.nowSeconds();
    var stopped = new ChassisSpeeds();
    var result = trailblazer.update(RobotStateSnapshot.measured(now, currentPose, stopped, true));
    trailblazer.update(RobotStateSnapshot.measured(now, currentPose, stopped, false));
    return Optional.of(result.commandedFieldSpeeds());
  }

  /** Returns {@code true} when every step in the routine has completed. */
  public final boolean isFinished() {
    return stepIndex >= steps.size();
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

  /** Advances and executes the auto state sequence using the planner clock. */
  public final void periodic() {
    periodic(trailblazer.nowSeconds());
  }

  /** Advances and executes the auto state sequence for one 20 ms robot loop. */
  public final void periodic(double nowSeconds) {
    startedRunning = true;
    while (stepIndex < steps.size()) {
      var step = steps.get(stepIndex);
      if (!stepEntered) {
        step.enter(nowSeconds);
        if (step.isInstant()) {
          stepIndex++;
          continue;
        }
        stepEntered = true;
        step.periodic();
        step.flushMarkers();
        break;
      }
      step.flushMarkers();
      if (step.isComplete(nowSeconds)) {
        stepIndex++;
        stepEntered = false;
        continue;
      }
      step.periodic();
      break;
    }
  }

  /**
   * Samples the full stitched preview polyline across all driving steps for AdvantageScope field
   * visualization before enabling.
   */
  public final Pose2d[] previewPath(Pose2d fallbackStartPose) {
    Pose2d cursor = allianceStartPose.orElse(fallbackStartPose);
    var poses = new ArrayList<Pose2d>();
    for (var step : driveSteps) {
      var goals = step.allianceSegment.goals();
      boolean staticOnly = true;
      for (var goal : goals) {
        if (goal.dynamicTarget() != null) {
          staticOnly = false;
          break;
        }
      }
      if (!staticOnly) {
        for (var goal : goals) {
          if (goal.dynamicTarget() == null) {
            poses.add(goal.pose());
            cursor = goal.pose();
          }
        }
        continue;
      }
      int index = 0;
      while (index < goals.size()) {
        int end = index;
        while (end < goals.size() - 1 && goals.get(end).passType() == Goal.PassType.THROUGH) {
          end++;
        }
        var subGoals = goals.subList(index, end + 1);
        var terminalPose = subGoals.get(subGoals.size() - 1).pose();
        try {
          var previewClearance =
              step.disableClearance
                  ? null
                  : (step.allianceClearance != null
                      ? step.allianceClearance
                      : CompAutoField.clearance(alliance));
          var geometry =
              PathBuilder.build(cursor.getTranslation(), subGoals, 1.6, previewClearance);
          var sampled = Trailblazer.sampleSegmentPoses(cursor, geometry, subGoals, 0.10);
          int startI = poses.isEmpty() ? 0 : 1;
          for (int i = startI; i < sampled.length; i++) {
            poses.add(sampled[i]);
          }
        } catch (IllegalArgumentException ignored) {
          if (poses.isEmpty()) {
            poses.add(cursor);
          }
          for (var goal : subGoals) {
            poses.add(goal.pose());
          }
        }
        cursor = terminalPose;
        index = end + 1;
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

  private void addStep(Step step) {
    if (pendingBranchSteps != null) {
      pendingBranchSteps.add(step);
      return;
    }
    checkState(!startedRunning, "Cannot add auto steps after the routine has started");
    steps.add(step);
  }

  /** Runs a one-shot action immediately and moves right into the next step. */
  protected final void action(String stepName, Runnable onceAction) {
    addStep(new ActionStep(stepName, onceAction));
  }

  /** Executes a nested sequence of steps when {@code condition} is true at branch entry. */
  protected final void branch(String stepName, BooleanSupplier condition, Runnable ifTrue) {
    branch(stepName, condition, ifTrue, () -> {});
  }

  /**
   * Chooses between two step sequences (supporting {@link #drive}, {@link #waitFor}, {@link
   * #waitSeconds}, {@link #action}, and nested {@link #branch}) when this step is reached.
   */
  protected final void branch(
      String stepName, BooleanSupplier condition, Runnable ifTrue, Runnable ifFalse) {
    addStep(new BranchStep(stepName, condition, ifTrue, ifFalse));
  }

  /** Drives along a Blue-alliance {@link AutoSegment} until reaching the goal. */
  protected final DriveStep drive(String stepName, AutoSegment blueSegment) {
    var step = new DriveStep(stepName, blueSegment);
    if (pendingBranchSteps == null) {
      if (steps.isEmpty()) {
        firstDriveStep = step;
      }
      driveSteps.add(step);
    }
    addStep(step.asStep());
    return step;
  }

  /**
   * Drives along a Blue-alliance {@link AutoSegment} checked against a Blue {@link FieldClearance}.
   */
  protected final DriveStep drive(
      String stepName, AutoSegment blueSegment, FieldClearance blueClearance) {
    return drive(stepName, blueSegment).withClearance(blueClearance);
  }

  /**
   * Holds position and waits until {@code condition} becomes true or {@code timeoutSeconds}
   * elapses.
   */
  protected final WaitStep waitFor(
      String stepName, double timeoutSeconds, BooleanSupplier condition) {
    var step = new WaitStep(stepName, timeoutSeconds, condition);
    addStep(step.asStep());
    return step;
  }

  /** Holds position for a fixed duration in seconds. */
  protected final WaitStep waitSeconds(String stepName, double durationSeconds) {
    return waitFor(stepName, durationSeconds, () -> false);
  }
}
