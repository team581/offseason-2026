package frc.robot.testing;

public record TestResult(String name, TestStatus status, String reason) {
  public boolean finished() {
    return status != TestStatus.RUNNING;
  }
}
