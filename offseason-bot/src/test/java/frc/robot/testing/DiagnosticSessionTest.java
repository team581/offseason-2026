package frc.robot.testing;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;
import org.junit.jupiter.api.Test;

final class DiagnosticSessionTest {
  private static final class FakeRoutine implements DiagnosticRoutine {
    int ticks;
    int stops;
    boolean throwOnTick;
    boolean throwOnAbort;
    ChassisSpeeds output = new ChassisSpeeds(0.5, 0, 0);
    Result result = new Result("fake", Status.RUNNING, "Running");

    @Override
    public void abort() {
      if (throwOnAbort) {
        throw new IllegalStateException("injected cleanup failure");
      }
      if (!result.finished()) {
        result = new Result("fake", Status.ABORTED, "Interrupted");
      }
      stop();
    }

    @Override
    public ChassisSpeeds requestedSpeeds() {
      return output;
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
      if (throwOnTick) {
        throw new IllegalStateException("injected failure");
      }
    }
  }

  private ChassisSpeeds output = new ChassisSpeeds();
  private int creations;
  private int prepareCalls;

  private DiagnosticSession session(FakeRoutine routine) {
    return new DiagnosticSession(
        () -> {
          creations++;
          return Optional.of(routine);
        },
        () -> prepareCalls++,
        speeds -> output = speeds);
  }

  @Test
  void cleanupExceptionStillClearsOutputAndLeavesTerminalEvidence() {
    var routine = new FakeRoutine();
    var session = session(routine);
    session.tick(true);
    routine.throwOnAbort = true;
    session.tick(false);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(session.state()).isEqualTo(DiagnosticSession.State.INACTIVE);
    assertThat(session.result().orElseThrow().status()).isEqualTo(DiagnosticRoutine.Status.FAILED);
    assertThat(session.reason()).contains("cleanup failure");
  }

  @Test
  void constructionFailureAlsoWritesZero() {
    output = new ChassisSpeeds(1, 0, 0);
    var session =
        new DiagnosticSession(
            () -> {
              throw new IllegalStateException("factory failed");
            },
            () -> {},
            speeds -> output = speeds);
    session.tick(true);
    assertThat(session.result().orElseThrow().reason()).contains("factory failed");
    assertThat(output.vxMetersPerSecond).isZero();
  }

  @Test
  void disabledDoesNotConstructPrepareOrTick() {
    var routine = new FakeRoutine();
    var session = session(routine);
    session.tick(false);
    assertThat(creations).isZero();
    assertThat(prepareCalls).isZero();
    assertThat(routine.ticks).isZero();
  }

  @Test
  void exceptionsAndInvalidOutputsCannotLeaveLastDriveCommandActive() {
    var routine = new FakeRoutine();
    var session = session(routine);
    session.tick(true);
    routine.throwOnTick = true;
    session.tick(true);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(session.result().orElseThrow().status()).isEqualTo(DiagnosticRoutine.Status.FAILED);
    routine.throwOnTick = false;
    routine.output = new ChassisSpeeds(Double.NaN, 0, 0);
    session.tick(false);
    routine.result =
        new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.RUNNING, "Running");
    session.tick(true);
    assertThat(session.result().orElseThrow().reason()).contains("nonfinite");
    assertThat(output.vxMetersPerSecond).isZero();
  }

  @Test
  void invalidConfigurationIsLatchedUntilNextEnable() {
    var session =
        new DiagnosticSession(
            () -> {
              creations++;
              throw new IllegalArgumentException("invalid limit");
            },
            () -> {},
            speeds -> output = speeds);
    session.tick(true);
    assertThat(session.state()).isEqualTo(DiagnosticSession.State.INVALID_CONFIG);
    session.tick(true);
    assertThat(creations).isEqualTo(1);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(session.results().get(0).reason()).isEqualTo("invalid limit");
  }

  @Test
  void snapshotsFactoryAndClearsOutputOnDisable() {
    var routine = new FakeRoutine();
    var session = session(routine);
    session.tick(true);
    session.tick(true);
    assertThat(creations).isEqualTo(1);
    assertThat(output.vxMetersPerSecond).isEqualTo(0.5);
    session.tick(false);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(session.result().orElseThrow().status()).isEqualTo(DiagnosticRoutine.Status.ABORTED);
    assertThat(routine.stops).isEqualTo(1);
    assertThat(session.state()).isEqualTo(DiagnosticSession.State.INACTIVE);
    session.tick(false);
    assertThat(routine.stops).isEqualTo(1);
    session.tick(true);
    assertThat(creations).isEqualTo(2);
  }

  @Test
  void terminalResultStopsImmediatelyAndNeverRerunsUntilReenabled() {
    var routine = new FakeRoutine();
    var session = session(routine);
    session.tick(true);
    routine.result =
        new DiagnosticRoutine.Result("fake", DiagnosticRoutine.Status.FAILED, "Too fast");
    session.tick(true);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(session.state()).isEqualTo(DiagnosticSession.State.FINISHED);
    int ticks = routine.ticks;
    session.tick(true);
    assertThat(routine.ticks).isEqualTo(ticks);
    session.tick(false);
    assertThat(session.result().orElseThrow().status()).isEqualTo(DiagnosticRoutine.Status.FAILED);
  }
}
