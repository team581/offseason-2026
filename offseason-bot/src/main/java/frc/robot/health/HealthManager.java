package frc.robot.health;

import com.team581.config.BlinkingBooleanBox;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.function.BooleanSupplier;

public class HealthManager extends StateMachineSubsystem<HealthState> {
  /**
   * Sparse tag sightings make the raw localization check flicker, which swaps aiming between the
   * estimated pose and the tower fallback every loop. Hold healthy briefly after the last good
   * check so a single missed frame doesn't move the turret.
   */
  private static final double LOCALIZATION_HEALTHY_HOLD_SECONDS = 1.0;

  private boolean localizationHealthy = false;
  private double lastLocalizationHealthyTimestamp = Double.NEGATIVE_INFINITY;
  private boolean fuelDetectionHealthy = false;
  private boolean allCamerasHealthy = false;

  private final BlinkingBooleanBox localizationBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/LocalizationHealthyBox", false, true);
  private final BlinkingBooleanBox allCamerasBlinkingBooleanBox =
      new BlinkingBooleanBox("Health/AllCamerasHealthyBox", false, true);

  private final Vision vision;
  private final boolean simulation;
  private BooleanSupplier poseTrustworthy = () -> false;

  public HealthManager(Vision vision) {
    this(vision, RobotBase.isSimulation());
  }

  /** Allows desktop tests to exercise the real robot's vision requirements. */
  public HealthManager(Vision vision, boolean simulation) {
    super(SubsystemPriority.HEALTH, HealthState.DEFAULT_STATE);
    this.vision = vision;
    this.simulation = simulation;
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

  public void setPoseTrustworthySupplier(BooleanSupplier supplier) {
    poseTrustworthy = supplier;
  }

  @Override
  protected void collectInputs() {
    var status = vision.getStatus();
    double now = vision.tickTime();
    if (vision.hasRecentFusedMeasurement() && poseTrustworthy.getAsBoolean()) {
      lastLocalizationHealthyTimestamp = now;
    }
    // Desktop simulation has odometry but no camera frames. Use its pose for aiming rather
    // than forcing RobotManager to aim from the fixed real-robot fallback location.
    localizationHealthy =
        simulation || now - lastLocalizationHealthyTimestamp <= LOCALIZATION_HEALTHY_HOLD_SECONDS;
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
