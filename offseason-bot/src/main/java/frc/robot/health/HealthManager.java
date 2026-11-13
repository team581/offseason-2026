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
   * A single noisy measurement can briefly push the trust factor over its threshold, which would
   * swap aiming between the estimated pose and the tower fallback. Hold healthy briefly after the
   * last trustworthy check so one bad frame doesn't move the turret.
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
    double now = vision.tickTime();
    // Trust is earned from fused Spot measurements and only decays while driving, so a robot
    // sitting where its cameras can't see a tag keeps the pose it last confirmed.
    if (poseTrustworthy.getAsBoolean()) {
      lastLocalizationHealthyTimestamp = now;
    }
    // Desktop simulation has odometry but no camera frames. Use its pose for aiming rather
    // than forcing RobotManager to aim from the fixed real-robot fallback location.
    localizationHealthy =
        simulation || now - lastLocalizationHealthyTimestamp <= LOCALIZATION_HEALTHY_HOLD_SECONDS;
    fuelDetectionHealthy = false;
    allCamerasHealthy = vision.isSpotReady();
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
