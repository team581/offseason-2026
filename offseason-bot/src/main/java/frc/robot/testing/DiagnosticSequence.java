package frc.robot.testing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/** Bounded, fail-fast test sequence. Completed results survive cleanup and disable. */
public final class DiagnosticSequence implements DiagnosticRoutine {
  public record Step(String name, double timeoutSeconds, Supplier<DiagnosticRoutine> factory) {
    public Step {
      if (name.isBlank() || !Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0) {
        throw new IllegalArgumentException("Each diagnostic needs a name and positive deadline");
      }
    }
  }

  private final List<Step> steps;
  private final List<Result> completed = new ArrayList<>();
  private Optional<DiagnosticRoutine> active = Optional.empty();
  private double startedAt;
  private boolean stopped;
  private Result result = new Result("Sequence", Status.RUNNING, "Waiting for first step");

  public DiagnosticSequence(List<Step> steps) {
    if (steps.isEmpty() || steps.stream().map(Step::name).distinct().count() != steps.size()) {
      throw new IllegalArgumentException("Sequence steps must be nonempty and uniquely named");
    }
    this.steps = List.copyOf(steps);
  }

  @Override
  public void abort() {
    if (!result.finished()) {
      if (active.isPresent()) {
        active.orElseThrow().abort();
        finishStep(
            new Result(
                steps.get(completed.size()).name(), Status.ABORTED, "Test mode interrupted"));
      }
      result = new Result("Sequence", Status.ABORTED, "Test mode interrupted");
    }
    stop();
  }

  @Override
  public ChassisSpeeds requestedSpeeds() {
    return stopped
        ? new ChassisSpeeds()
        : active.map(DiagnosticRoutine::requestedSpeeds).orElseGet(ChassisSpeeds::new);
  }

  @Override
  public Result result() {
    return result;
  }

  @Override
  public List<Result> results() {
    var results = new ArrayList<>(completed);
    for (int i = completed.size(); i < steps.size(); i++) {
      var step = steps.get(i);
      results.add(
          i == completed.size() && active.isPresent()
              ? new Result(
                  step.name(),
                  active.orElseThrow().result().status(),
                  active.orElseThrow().result().reason())
              : new Result(
                  step.name(),
                  result.finished() ? Status.BLOCKED : Status.RUNNING,
                  result.finished()
                      ? "Earlier diagnostic failed or run was interrupted"
                      : "Not started"));
    }
    return List.copyOf(results);
  }

  @Override
  public void stop() {
    stopped = true;
    active.ifPresent(DiagnosticRoutine::stop);
  }

  @Override
  public void tick() {
    if (result.finished() || stopped) {
      return;
    }
    if (active.isEmpty()) {
      var step = steps.get(completed.size());
      active = Optional.of(step.factory().get());
      startedAt = Timer.getFPGATimestamp();
    }
    var routine = active.orElseThrow();
    var step = steps.get(completed.size());
    if (Timer.getFPGATimestamp() - startedAt >= step.timeoutSeconds()) {
      finishStep(new Result(step.name(), Status.FAILED, "Diagnostic deadline exceeded"));
      return;
    }
    routine.tick();
    if (routine.result().finished()) {
      finishStep(new Result(step.name(), routine.result().status(), routine.result().reason()));
    }
  }

  private void finishStep(Result stepResult) {
    active.orElseThrow().stop();
    active = Optional.empty();
    completed.add(stepResult);
    if (stepResult.status() != Status.PASSED) {
      result =
          new Result(
              "Sequence", stepResult.status(), stepResult.name() + ": " + stepResult.reason());
    } else if (completed.size() == steps.size()) {
      result = new Result("Sequence", Status.PASSED, "All " + steps.size() + " diagnostics passed");
    }
  }
}
