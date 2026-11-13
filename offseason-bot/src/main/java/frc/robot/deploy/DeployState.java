package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(-5.0),
  STOW(20),
  // Initial angle targets in degrees; tune during bringup.
  SCORE_COMPACTION_WAITING(19.3),
  SCORE_COMPACTION(20),
  FEED_COMPACTION(37.0),

  BEAST_MODE_COMPACTION(20),

  AUTO_INTAKE_DELAY(DeployConfig.HOMING_END_POSITION_INWARD),
  HOME_INWARD(DeployConfig.HOMING_END_POSITION_INWARD),
  HOME_OUTWARD(-5.0),
  UNHOMED(DeployConfig.HOMING_END_POSITION_INWARD);

  private final DoubleSubscriber tunableAngle;

  DeployState(double angleDegrees) {

    this.tunableAngle = DogLog.tunable("Deploy/State/" + name() + "/AngleDegrees", angleDegrees);
  }

  /** Target arm angle in degrees: zero is fully down, positive is raised. */
  public double getAngleDegrees() {
    return tunableAngle.get();
  }

  public boolean isHoming() {
    return switch (this) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> true;
      default -> false;
    };
  }
}
