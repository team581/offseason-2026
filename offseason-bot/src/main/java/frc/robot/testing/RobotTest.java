package frc.robot.testing;

import static java.util.UUID.randomUUID;

import com.team581.util.state_machines.StateMachine;

/** A robot diagnostic state machine run by TestManager. */
public abstract class RobotTest<S extends Enum<S>> extends StateMachine<S> {
  private final String runId = randomUUID().toString();

  protected RobotTest(S initialState) {
    super(initialState);
  }

  public abstract void abort();

  public final String getRunId() {
    return runId;
  }

  public abstract TestResult result();
}
