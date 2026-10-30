package frc.robot.deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  INTAKE(DeployConfig.MAX_LENGTH),
  STOW(0.0), // For The stow state Set to 0.0 because (fully retracted)
  //  Compaction & Scoring States
  SCORE_READY(DeployConfig.MAX_LENGTH - 2.5), // Move Back 2.5 INCHES
  SCORE_COMPACTION(0.0), // TBD
  FEED_COMPACTION(0.0), // TBD

  FIX_ALIGHNMENT(DeployConfig.MAX_LENGTH),

  // Homing States
  HOME_INWARD(0.0),
  HOME_OUTWARD(DeployConfig.MAX_LENGTH), // Hardstop target is physical maximum (12.75")
  UNHOMED(0.0);

  private final DoubleSubscriber tunableLength;

  DeployState(double defaultLength) {
    this.tunableLength = DogLog.tunable("Deploy/State/" + name(), defaultLength);
  }

  public double getLength() {
    return tunableLength.get();
  }

  public boolean isHoming() {
    return switch (this) {
      case UNHOMED, HOME_INWARD, HOME_OUTWARD -> true;
      default -> false;
    };
  }
}
