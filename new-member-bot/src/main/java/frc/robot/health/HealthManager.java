package frc.robot.health;

import com.team581.config.BlinkingBooleanBox;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;

public class HealthManager extends StateMachineSubsystem<HealthState> {
  private final Limelight frontLimelight;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;

  private boolean localizationHealthy = true;
  private boolean allCamerasHealthy = true;

  private final BlinkingBooleanBox localizationBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/LocalizationHealthyBox", false, true);
  private final BlinkingBooleanBox allCamerasBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/AllCamerasHealthyBox", false, true);

  public HealthManager(
      Limelight frontLimelight, Limelight leftLimelight, Limelight rightLimelight) {
    super(SubsystemPriority.HEALTH, HealthState.DEFAULT_STATE);

    this.frontLimelight = frontLimelight;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
  }

  /** Returns whether all cameras are healthy. */
  public boolean isAllCamerasHealthy() {
    return allCamerasHealthy;
  }

  /** Returns whether the robot's ability to localize itself is healthy. */
  public boolean isLocalizationHealthy() {
    return localizationHealthy
        && DSOptions.USE_TAG_LIMELIGHTS.getAsBoolean()
        && !DSOptions.PIT_FUNCTIONALITY.getAsBoolean();
  }

  @Override
  protected void collectInputs() {
    localizationHealthy =
        RobotBase.isSimulation()
            || frontLimelight.getCameraHealth() != CameraHealth.OFFLINE
            || leftLimelight.getCameraHealth() != CameraHealth.OFFLINE
            || rightLimelight.getCameraHealth() != CameraHealth.OFFLINE;
    allCamerasHealthy =
        RobotBase.isSimulation()
            || (frontLimelight.getCameraHealth() != CameraHealth.OFFLINE
                && leftLimelight.getCameraHealth() != CameraHealth.OFFLINE
                && rightLimelight.getCameraHealth() != CameraHealth.OFFLINE);
  }

  @Override
  protected void whileInState(HealthState state) {
    DogLog.log("Health/LocalizationHealthy", localizationHealthy);
    DogLog.log("Health/AllCamerasHealthy", allCamerasHealthy);

    localizationBlinkingBooleanBox.update(localizationHealthy);
    allCamerasBlinkingBooleanBox.update(allCamerasHealthy);
  }
}
