package frc.robot.health;

import com.team581.config.BlinkingBooleanBox;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class HealthManager extends StateMachineSubsystem<HealthState> {

  private boolean localizationHealthy = false;
  private boolean fuelDetectionHealthy = false;
  private boolean allCamerasHealthy = false;

  private final BlinkingBooleanBox localizationBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/LocalizationHealthyBox", false, true);
  private final BlinkingBooleanBox allCamerasBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/AllCamerasHealthyBox", false, true);

  private final Vision vision;

  public HealthManager(Vision vision) {
    super(SubsystemPriority.HEALTH, HealthState.DEFAULT_STATE);
    this.vision = vision;
  }

  /** Returns whether all cameras are healthy. */
  public boolean isAllCamerasHealthy() {
    return allCamerasHealthy;
  }

  /** Returns whether the robot's ability to detect fuel is healthy. */
  public boolean isFuelDetectionHealthy() {
    return fuelDetectionHealthy;
  }

  /** Returns whether the robot's ability to localize itself is healthy. */
  public boolean isLocalizationHealthy() {
    return localizationHealthy
        && DSOptions.USE_VISION.getAsBoolean()
        && !DSOptions.PIT_FUNCTIONALITY.getAsBoolean();
  }

  @Override
  protected void collectInputs() {
    var status = vision.getStatus();
    localizationHealthy = status.available() && status.usable() > 0;
    fuelDetectionHealthy = false;
    allCamerasHealthy = status.allRequiredHealthy();
  }

  @Override
  protected void whileInState(HealthState state) {
    DogLog.log("Health/LocalizationHealthy", localizationHealthy);
    DogLog.log("Health/FuelDetectionHealthy", fuelDetectionHealthy);
    DogLog.log("Health/AllCamerasHealthy", allCamerasHealthy);

    localizationBlinkingBooleanBox.update(localizationHealthy);
    allCamerasBlinkingBooleanBox.update(allCamerasHealthy);
  }
}
