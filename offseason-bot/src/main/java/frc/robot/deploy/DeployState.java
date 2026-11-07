package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(0.0),
  INTAKE_COAST(0.0),
  INTAKE_COLLAPSED(0.0),
  INTAKE_RECOVERING(0.0),
  INTAKE_RECOVERY_FAILED(0.0),
  STOW(45),
  // Initial angle targets in degrees; tune during bringup.
  SCORE_COMPACTION_WAITING(19.3),
  SCORE_COMPACTION(45),
  FEED_COMPACTION(37.0),

  BEAST_MODE_COMPACTION(45),

  FIX_DIFFERENTIAL_DESYNC(0.0),

  AUTO_INTAKE_DELAY(DeployConfig.HOMING_END_POSITION_INWARD),
  HOME_INWARD(DeployConfig.HOMING_END_POSITION_INWARD),
  HOME_OUTWARD(0.0),
  UNHOMED(DeployConfig.HOMING_END_POSITION_INWARD);

  private final DoubleSubscriber tunableAngle;

  DeployState(double angleDegrees) {

    this.tunableAngle = DogLog.tunable("Deploy/State/" + name() + "/AngleDegrees", angleDegrees);
  }

  /** Target arm angle in degrees: zero is fully down, positive is raised. */
  public double getAngleDegrees() {
    return this != INTAKE && isIntaking() ? INTAKE.getAngleDegrees() : tunableAngle.get();
  }

  public boolean isCoasting() {
    return this == INTAKE_COAST || this == INTAKE_COLLAPSED || this == INTAKE_RECOVERY_FAILED;
  }

  public boolean isHoming() {
    return switch (this) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD, AUTO_INTAKE_DELAY -> true;
      default -> false;
    };
  }

  public boolean isIntaking() {
    return switch (this) {
      case INTAKE, INTAKE_COAST, INTAKE_COLLAPSED, INTAKE_RECOVERING, INTAKE_RECOVERY_FAILED ->
          true;
      default -> false;
    };
  }
}
