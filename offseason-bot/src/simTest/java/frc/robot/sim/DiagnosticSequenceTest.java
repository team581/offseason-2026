package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.testing.DiagnosticRoutine;
import frc.robot.testing.DiagnosticSequence;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class DiagnosticSequenceTest {
  private static final class FakeRoutine implements DiagnosticRoutine {
    int stops;
    int ticks;
    Result result = new Result("fake", Status.RUNNING, "Running");

    @Override
    public void abort() {
      result = new Result("fake", Status.ABORTED, "Aborted");
    }

    @Override
    public ChassisSpeeds requestedSpeeds() {
      return new ChassisSpeeds(0.5, 0, 0);
    }

    @Override
    public Result result() {
      return result;
    }

    @Override
    public void stop() {
      stops++;
    }

    @Override
    public void tick() {
      ticks++;
    }
  }

  @Test
  void abortKeepsEarlierResultsAndStopsActiveStep() {
    var first = new FakeRoutine();
    first.result = new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.PASSED, "Done");
    var second = new FakeRoutine();
    var sequence =
        new DiagnosticSequence(
            List.of(
                new DiagnosticSequence.Step("first", 1, () -> first),
                new DiagnosticSequence.Step("second", 1, () -> second)));
    sequence.tick();
    sequence.tick();
    sequence.abort();
    assertThat(sequence.results())
        .extracting(DiagnosticRoutine.Result::status)
        .containsExactly(DiagnosticRoutine.Status.PASSED, DiagnosticRoutine.Status.ABORTED);
    assertThat(sequence.requestedSpeeds().vxMetersPerSecond).isZero();
  }

  @AfterEach
  void cleanup() {
    SimHooks.resumeTiming();
  }

  @Test
  void deadlineFailsAndStopsAnOtherwiseUnboundedRoutine() {
    var routine = new FakeRoutine();
    var sequence =
        new DiagnosticSequence(List.of(new DiagnosticSequence.Step("stuck", 1, () -> routine)));
    sequence.tick();
    SimHooks.stepTimingAsync(1.01);
    sequence.tick();
    assertThat(sequence.result().status()).isEqualTo(DiagnosticRoutine.Status.FAILED);
    assertThat(sequence.result().reason()).contains("deadline");
    assertThat(routine.stops).isEqualTo(1);
  }

  @Test
  void failedStepStopsAndBlocksDependentStepsWithoutConstructingThem() {
    var first = new FakeRoutine();
    first.result =
        new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.FAILED, "Too fast");
    var second = new FakeRoutine();
    var sequence =
        new DiagnosticSequence(
            List.of(
                new DiagnosticSequence.Step("first", 1, () -> first),
                new DiagnosticSequence.Step("second", 1, () -> second)));
    sequence.tick();
    assertThat(first.stops).isEqualTo(1);
    assertThat(second.ticks).isZero();
    assertThat(sequence.results())
        .extracting(DiagnosticRoutine.Result::status)
        .containsExactly(DiagnosticRoutine.Status.FAILED, DiagnosticRoutine.Status.BLOCKED);
    assertThat(sequence.requestedSpeeds().vxMetersPerSecond).isZero();
  }

  @Test
  void passingStepsAreSeparatedByStoppedOutputAndAllMustPass() {
    var first = new FakeRoutine();
    first.result = new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.PASSED, "Done");
    var second = new FakeRoutine();
    var sequence =
        new DiagnosticSequence(
            List.of(
                new DiagnosticSequence.Step("first", 1, () -> first),
                new DiagnosticSequence.Step("second", 1, () -> second)));
    sequence.tick();
    assertThat(sequence.result().status()).isEqualTo(DiagnosticRoutine.Status.RUNNING);
    assertThat(sequence.requestedSpeeds().vxMetersPerSecond).isZero();
    sequence.tick();
    assertThat(sequence.requestedSpeeds().vxMetersPerSecond).isEqualTo(0.5);
    second.result = new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.PASSED, "Done");
    sequence.tick();
    assertThat(sequence.result().status()).isEqualTo(DiagnosticRoutine.Status.PASSED);
    assertThat(second.stops).isEqualTo(1);
  }

  @BeforeEach
  void setup() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    SimHooks.pauseTiming();
  }
}
