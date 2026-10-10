package frc.robot.testing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;

/** Test-owned protocol. Production managers and mechanisms do not implement this interface. */
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
    return List.of(result());
  }

  /** Must be safe before the first tick and after any terminal result. */
  void stop();

  /** Called once per enabled Test-mode loop. */
  void tick();
}
