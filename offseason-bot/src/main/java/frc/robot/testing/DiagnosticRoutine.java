package frc.robot.testing;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

/** A diagnostic that produces drive commands and a pass/fail result. */
public interface DiagnosticRoutine {
  record Result(String name, Status status, String reason) {
    public boolean finished() {
      return status != Status.RUNNING;
    }
  }

  enum Status {
    RUNNING,
    PASSED,
    FAILED,
    ABORTED,
    BLOCKED
  }

  void abort();

  ChassisSpeeds requestedSpeeds();

  Result result();

  default List<Result> results() {
    return ImmutableList.of(result());
  }

  /** Clears drive commands; safe to call before starting or after finishing. */
  void stop();

  /** Advances the diagnostic once per enabled Test-mode loop. */
  void tick();
}
