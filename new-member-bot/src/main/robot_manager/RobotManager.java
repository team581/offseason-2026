package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.health.HealthManager;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Shooter shooter;
  // private final ShooterHood shooterHood;//add when time comes
  public final Swerve swerve;
  // public final HopperManager hopperMananger;//add when time comes
  public final HealthManager healthManager;
  public final Localization localization;
  // public final PowerManager powerManager;// add when time comes
  public final XboxController driverController;
  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0, 0);

  public RobotManager(
      HopperManager hopperManager,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      XboxController driverController,
      HealthManager health,
      Hardware hardware) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.hopperManager = hopperManager;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.driverController = driverController;
    this.health = health;
    this.hardware = hardware;
    this.powerManager = powerManager;
  }

  public void prepareFeedRequest() {
    if (getState() != RobotState.FALLBACK_FEED && getState() != RobotState.FEED) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_FALLBACK_FEED);
      } else {
        setStateFromRequest(RobotState.PREPARE_FEED);
      }
    }
  }

  public void prepareScoreOrFeedRequest() {
    if (FeatureFlags.BRING_UP.getAsBoolean()) {
      forceShootRequest();
      return;
    }
    if (isInAllianceZone) {
      prepareScoreRequest();
    } else {
      prepareFeedRequest();
    }
  }

  public void prepareScoreRequest() {
    if (getState() != RobotState.FALLBACK_SCORE && getState() != RobotState.SCORE) {
      if (!health.isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_FALLBACK_SCORE);
      } else {
        setStateFromRequest(RobotState.PREPARE_SCORE);
      }
    }
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    idleRequest();
  }

  public void unjamRequest() {
    setStateFromRequest(RobotState.UNJAM);
  }

  public void warmupScoreOrFeedRequest() {
    if (isInAllianceZone) {
      warmupScoreRequest();
    } else {
      warmupFeedRequest();
    }
  }

  public void warmupScoreRequest() {
    switch (getState()) {
      case SCORE, PREPARE_SCORE, PREPARE_FALLBACK_SCORE, FALLBACK_SCORE -> {}
      default -> {
        setStateFromRequest(RobotState.WARMUP_SCORE);
      }
    }
  }

  private void logFeedTransition() {
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/ShooterAtGoalDebounced", shooter.atGoalDebounced());
    DogLog.log("RobotManager/Feeding/FeedTransition/SafeFeedLocation", isInSafeFeedingLocation);
    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/LocalizationHealthy", health.isLocalizationHealthy());
    DogLog.log("RobotManager/Feeding/ScoreTransition/NotNearTrench", !nearTrench);
    DogLog.log("RobotManager/Feeding/FeedTransition/IsMoving", isMoving);
  }
}
