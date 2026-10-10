package frc.robot.testing;

import static com.google.common.base.Preconditions.checkState;
import static java.util.Objects.requireNonNull;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Enable-edge lifecycle shared by diagnostic adapters, never by production robot managers. */
public final class DiagnosticSession {
  public enum State {
    INACTIVE,
    RUNNING,
    FINISHED,
    INVALID_CONFIG
  }

  private final Supplier<Optional<DiagnosticRoutine>> factory;
  private final Runnable prepare;
  private final Consumer<ChassisSpeeds> output;
  private Optional<DiagnosticRoutine> active = Optional.empty();
  private boolean previouslyEnabled;
  private State state = State.INACTIVE;
  private String reason = "Waiting for Test mode enable";
  private Optional<DiagnosticRoutine.Result> failure = Optional.empty();

  public DiagnosticSession(
      Supplier<Optional<DiagnosticRoutine>> factory,
      Runnable prepare,
      Consumer<ChassisSpeeds> output) {
    this.factory = factory;
    this.prepare = prepare;
    this.output = output;
  }

  public Optional<DiagnosticRoutine> active() {
    return active;
  }

  public String reason() {
    return reason;
  }

  public Optional<DiagnosticRoutine.Result> result() {
    return failure.or(() -> active.map(DiagnosticRoutine::result));
  }

  public List<DiagnosticRoutine.Result> results() {
    return failure.isPresent()
        ? ImmutableList.of(failure.orElseThrow())
        : active.map(DiagnosticRoutine::results).orElseGet(List::of);
  }

  public State state() {
    return state;
  }

  public void tick(boolean enabled) {
    boolean entering = enabled && !previouslyEnabled;
    boolean leaving = !enabled && previouslyEnabled;
    previouslyEnabled = enabled;
    if (!enabled) {
      if (leaving) {
        try {
          active.ifPresent(DiagnosticRoutine::abort);
        } catch (RuntimeException exception) {
          reason = "Diagnostic cleanup exception: " + exception;
          failure =
              Optional.of(
                  new DiagnosticRoutine.Result("Session", DiagnosticRoutine.Status.FAILED, reason));
        } finally {
          state = State.INACTIVE;
          output.accept(new ChassisSpeeds());
        }
      }
      return;
    }
    // Default to zero even if preparation, construction or a routine throws.
    var requested = new ChassisSpeeds();
    try {
      if (entering) {
        active = Optional.empty();
        failure = Optional.empty();
        try {
          // Factory construction snapshots selection and configuration once per enable.
          active = requireNonNull(factory.get());
          state = active.isPresent() ? State.RUNNING : State.INACTIVE;
          reason = active.isPresent() ? "Running" : "No diagnostic selected";
        } catch (IllegalArgumentException exception) {
          state = State.INVALID_CONFIG;
          reason = exception.getMessage();
          failure =
              Optional.of(
                  new DiagnosticRoutine.Result(
                      "Configuration", DiagnosticRoutine.Status.FAILED, reason));
        }
      }
      prepare.run();
      if (state == State.RUNNING) {
        var routine = active.orElseThrow();
        routine.tick();
        reason = routine.result().reason();
        if (routine.result().finished()) {
          state = State.FINISHED;
          routine.stop();
        } else {
          var next = routine.requestedSpeeds();
          checkState(
              Double.isFinite(next.vxMetersPerSecond)
                  && Double.isFinite(next.vyMetersPerSecond)
                  && Double.isFinite(next.omegaRadiansPerSecond),
              "Diagnostic requested nonfinite drive output");
          requested = next;
        }
      }
    } catch (RuntimeException exception) {
      state = State.FINISHED;
      reason = "Diagnostic exception: " + exception;
      failure =
          Optional.of(
              new DiagnosticRoutine.Result("Session", DiagnosticRoutine.Status.FAILED, reason));
      try {
        active.ifPresent(DiagnosticRoutine::abort);
      } catch (RuntimeException cleanupException) {
        reason += "; cleanup exception: " + cleanupException;
        failure =
            Optional.of(
                new DiagnosticRoutine.Result("Session", DiagnosticRoutine.Status.FAILED, reason));
      }
    } finally {
      output.accept(requested);
    }
  }
}
