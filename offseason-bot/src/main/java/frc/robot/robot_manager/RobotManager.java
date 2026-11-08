package frc.robot.robot_manager;

import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Hardware;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.shooter_hood.ShooterHoodState;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretState;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final HopperManager hopperManager;
  public final Localization localization;
  public final Swerve swerve;
  public final Turret turret;
  public final Hardware hardware;
  private final ShooterHood shooterHood;
  public final Shooter shooter;
  private final Vision vision;
  public final XboxController driverController;
  private final HealthManager health;
  private final HubActivity hubActivity;

  public final PowerManager powerManager;
  private Pose2d robotPose = Pose2d.kZero;

  private boolean nearTrench = false;
  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0, 0, 0);

  private AimingParameters fallbackFeedingParameters = new AimingParameters(0, 0, 0, 0, 0);
  private boolean isMoving = false;
  private boolean trenchOverride = false;

  private boolean isInAllianceZone = false;
  private boolean isInSafeFeedingLocation = true;
  private boolean isScorePathObstructed = false;

  // Number gotten from trench scoring point
  private DoubleSubscriber SLOW_SCORING_DISTANCE_THRESHOLD =
      DogLog.tunable("RobotManager/FarScoreThreshold", 4.0);

  private DoubleSubscriber SLOW_FEEDING_DISTANCE_THRESHOLD =
      DogLog.tunable("RobotManager/FarFeedThreshold", 8.5);

  private FeedLocation feedLocation = FeedLocation.RIGHT;

  public RobotManager(
      HopperManager hopperManager,
      ShooterHood shooterHood,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      Turret turret,
      Vision vision,
      XboxController driverController,
      HealthManager health,
      HubActivity hubActivity,
      Hardware hardware,
      PowerManager powerManager) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.hopperManager = hopperManager;
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.turret = turret;
    this.vision = vision;
    this.driverController = driverController;
    this.health = health;
    this.hubActivity = hubActivity;

    this.hardware = hardware;
    this.powerManager = powerManager;
  }

  public void cancelEjectRequest() {
    hopperManager.setDriverWantsEject(false);
  }

  public void cancelIntakeRequest() {
    hopperManager.setDriverWantsIntake(false);
  }

  public void cancelStowDeployRequest() {
    hopperManager.setOperatorWantsStow(false);
  }

  public void cancelWarmupRequest() {
    switch (getState()) {
      case SCORE,
          PREPARE_SCORE,
          PREPARE_FALLBACK_SCORE,
          FALLBACK_SCORE,
          FEED,
          PREPARE_FEED,
          PREPARE_FALLBACK_FEED,
          FALLBACK_FEED -> {}
      default -> {
        idleRequest();
      }
    }
  }

  public void ejectRequest() {
    hopperManager.setDriverWantsEject(true);
  }

  public void forceScoreRequest() {
    if (getState() != RobotState.FORCE_SCORE) {
      setStateFromRequest(RobotState.PREPARE_FORCE_SCORE);
    }
  }

  public void homeDeployInAutoRequest() {
    hopperManager.deploy.homeInAutoRequest();
  }

  public void homeDeployRequest() {
    hopperManager.deploy.homingRequest();
  }

  public void homeShooterHoodRequest() {
    shooterHood.homingRequest();
  }

  public void idleRequest() {
    setStateFromRequest(RobotState.IDLE);
  }

  public void intakeRequest() {
    hopperManager.setDriverWantsIntake(true);
  }

  public void prepareFallbackScoreRequest() {
    if (getState() != RobotState.FALLBACK_SCORE) {
      setStateFromRequest(RobotState.PREPARE_FALLBACK_SCORE);
    }
  }

  public void prepareFeedRequest() {
    if (getState() != RobotState.FALLBACK_FEED && getState() != RobotState.FEED) {
      if (!isLocalizationHealthy()) {
        setStateFromRequest(RobotState.PREPARE_FALLBACK_FEED);
      } else {
        setStateFromRequest(RobotState.PREPARE_FEED);
      }
    }
  }

  public void prepareScoreOrFeedRequest() {
    if (FeatureFlags.BRING_UP.getAsBoolean()) {
      forceScoreRequest();
      return;
    }
    if (isInAllianceZone) {
      prepareScoreRequest();
    } else {
      prepareFeedRequest();
    }
  }

  /**
   * Without healthy localization the shot waits in PREPARE_SCORE instead of silently switching to
   * the tower preset. The preset is only used when the driver explicitly requests it.
   */
  public void prepareScoreRequest() {
    if (getState() != RobotState.FALLBACK_SCORE && getState() != RobotState.SCORE) {
      setStateFromRequest(RobotState.PREPARE_SCORE);
    }
  }

  public void setTrenchOverrideRequest(boolean trenchOverride) {
    this.trenchOverride = trenchOverride;
  }

  public void stowDeployRequest() {
    hopperManager.setOperatorWantsStow(true);
  }

  @Override
  public void teleopInit() {
    super.teleopInit();
    idleRequest();
  }

  public void unjamRequest() {
    setStateFromRequest(RobotState.UNJAM);
  }

  public void warmupFeedRequest() {
    switch (getState()) {
      case FEED, PREPARE_FEED, PREPARE_FALLBACK_FEED, FALLBACK_FEED -> {}
      default -> {
        setStateFromRequest(RobotState.WARMUP_FEED);
      }
    }
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

  /**
   * Auto starts from a known pose and odometry stays accurate enough to aim through the routine, so
   * auto keeps aiming from its pose instead of the tower fallback when vision isn't working.
   */
  private boolean isLocalizationHealthy() {
    return DriverStation.isAutonomous() || health.isLocalizationHealthy();
  }

  private void logFallbackScoringTransition(ShooterState expectedShooterState) {
    DogLog.log(
        "RobotManager/FallbackScoring/ScoreTransition/TurretAtGoal",
        turret.atGoal(TurretState.SCORE, scoringParameters));
    DogLog.log(
        "RobotManager/FallbackScoring/ScoreTransition/ShooterAtGoalDebounced",
        shooter.atGoalDebounced(expectedShooterState));
    DogLog.log(
        "RobotManager/FallbackScoring/ScoreTransition/ShooterHoodAtGoal",
        shooterHood.atGoal(ShooterHoodState.SCORING));
  }

  private void logFeedTransition(RobotState currentState) {
    DogLog.log("RobotManager/Feeding/FeedTransition/NotInAllianceZone", !isInAllianceZone);
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/TurretAtGoal",
        turret.atGoal(TurretState.FEED, feedingParameters));
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/ShooterAtGoalDebounced",
        shooter.atGoalDebounced(
            currentState == RobotState.FEED ? ShooterState.FEED : ShooterState.PREPARE_FEED));
    DogLog.log("RobotManager/Feeding/FeedTransition/SafeFeedLocation", isInSafeFeedingLocation);
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal",
        shooterHood.atGoal(ShooterHoodState.FEEDING));
    DogLog.log("RobotManager/Feeding/FeedTransition/LocalizationHealthy", isLocalizationHealthy());
    DogLog.log("RobotManager/Feeding/FeedTransition/NotNearTrench", !nearTrench);
    DogLog.log("RobotManager/Feeding/FeedTransition/IsMoving", isMoving);
  }

  private void logScoringTransition(RobotState currentState) {
    DogLog.log("RobotManager/Scoring/ScoreTransition/InAllianceZone", isInAllianceZone);
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/TurretAtGoal",
        turret.atGoal(TurretState.SCORE, scoringParameters));
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/ShooterAtGoalDebounced",
        shooter.atGoalDebounced(
            currentState == RobotState.SCORE ? ShooterState.SCORE : ShooterState.PREPARE_SCORE));
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/ShooterHoodAtGoal",
        shooterHood.atGoal(ShooterHoodState.SCORING));
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/LocalizationTrustworthy",
        localization.isTrustworthy());
    DogLog.log("RobotManager/Scoring/ScoreTransition/LocalizationHealthy", isLocalizationHealthy());

    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/HubActive", hubActivity.getTOFBasedHubActive());
    DogLog.log("RobotManager/Scoring/ScoreTransition/NotNearTrench", !nearTrench);
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/SwerveSafeSpeed", !swerve.isMovingBeyondSafeSpeed());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/DriverNotDecidingSotm",
        !swerve.driverStillDecidingSotm());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/AccelerationLowEnough",
        localization.imu.accelerationLowEnoughToShoot());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/ScorePathNotObstructed", !isScorePathObstructed);
  }

  private void smartFeedingPowerManagerRequest() {
    if (robotPose.getTranslation().getDistance(feedLocation.getTranslation())
        > SLOW_FEEDING_DISTANCE_THRESHOLD.get()) {
      powerManager.feedingFarRequest();
    } else {
      powerManager.feedingRequest();
    }
  }

  private void smartHoodPrepareFallbackFeedRequest() {
    // In fallback feeding, stow if near trench
    if (nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/FallbackFeeding/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/FallbackFeeding/HoodStatus", "Clear");
      shooterHood.feedRequest(fallbackFeedingParameters.distance());
    }
  }

  private void smartHoodPrepareFeedRequest() {
    // If cameras are offline or we are near a trench, always be idle
    if (!isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Feeding/SmartPrepareFeed/HoodStatus", "NearTrenchOrUnhealthy");
    } else {
      DogLog.log("RobotManager/Feeding/SmartPrepareFeed/HoodStatus", "Clear");
      shooterHood.feedRequest(feedingParameters.distance());
    }
  }

  private void smartHoodPrepareScoreRequest() {
    // If cameras are offline or we are near a trench, always be idle
    if (!isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrenchOrUnhealthy");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "Clear");
      shooterHood.scoreRequest(scoringParameters.distance());
    }
  }

  private void smartScoringPowerManagerRequest() {
    if (hubActivity.shouldBeastMode()) {
      powerManager.beastModeRequest();
    } else if (robotPose.getTranslation().getDistance(FieldUtil.HUB_POSE.getTranslation())
        > SLOW_SCORING_DISTANCE_THRESHOLD.get()) {
      powerManager.scoringFarRequest();
    } else {
      powerManager.scoringRequest();
    }
  }

  private void smartTurretIdleRequest() {
    if (DriverStation.isAutonomous() || isInAllianceZone) {
      turret.idleScoreRequest(scoringParameters);
    } else if (!isLocalizationHealthy()) {
      turret.idleFeedRequest(fallbackFeedingParameters);
    } else {
      turret.idleFeedRequest(feedingParameters);
    }
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        hopperManager.idleRequest();
        vision.hubTagsRequest();
        shooter.idleRequest();
        shooterHood.idleRequest();
        smartTurretIdleRequest();
        swerve.normalDriveRequest();
        powerManager.idleRequest();
      }
      case PREPARE_FORCE_SCORE -> {
        hopperManager.idleRequest();
        vision.tagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        powerManager.scoringRequest();
      }
      case FORCE_SCORE -> {
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        vision.tagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        powerManager.scoringRequest();
      }
      case PREPARE_FEED -> {
        hopperManager.idleRequest();
        vision.tagsRequest();
        shooter.prepareFeedRequest(feedingParameters.distance());
        smartHoodPrepareFeedRequest();
        turret.feedRequest(feedingParameters);
        swerve.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case FEED -> {
        hopperManager.feedRequest();
        vision.tagsRequest();
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(feedingParameters);
        swerve.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case PREPARE_SCORE -> {
        hopperManager.idleRequest();
        vision.hubTagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        smartHoodPrepareScoreRequest();
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
      }
      case SCORE -> {
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        vision.hubTagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
      }
      case PREPARE_FALLBACK_FEED -> {
        hopperManager.idleRequest();
        vision.tagsRequest();
        shooter.prepareFeedRequest(fallbackFeedingParameters.distance());
        smartHoodPrepareFallbackFeedRequest();
        turret.feedRequest(fallbackFeedingParameters);
        swerve.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case FALLBACK_FEED -> {
        hopperManager.feedRequest();
        vision.tagsRequest();
        shooter.feedRequest(fallbackFeedingParameters.distance());
        shooterHood.feedRequest(fallbackFeedingParameters.distance());
        turret.feedRequest(fallbackFeedingParameters);
        swerve.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case PREPARE_FALLBACK_SCORE -> {
        hopperManager.idleRequest();
        vision.tagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
      }
      case FALLBACK_SCORE -> {
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        vision.tagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
      }
      case UNJAM -> {
        hopperManager.unjamRequest();
        vision.tagsRequest();
        shooter.scoreRequest(3);
        shooterHood.scoreRequest(3);
        smartTurretIdleRequest();
        swerve.normalDriveRequest();
        powerManager.idleRequest();
      }
      case WARMUP_SCORE -> {
        vision.hubTagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.idleRequest();
        turret.scoreRequest(scoringParameters);
        swerve.warmupScoreRequest();
        smartScoringPowerManagerRequest();
        hopperManager.idleRequest();
      }
      case WARMUP_FEED -> {
        vision.tagsRequest();
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.idleRequest();
        turret.feedRequest(feedingParameters);
        swerve.warmupFeedRequest();
        smartFeedingPowerManagerRequest();
        hopperManager.idleRequest();
      }
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    feedLocation = FeedLocation.closest(robotPose);
    var speeds = swerve.getFieldRelativeSpeeds();
    var velocity = MathHelpers.getLinearVelocity(speeds);

    isMoving = velocity > 0.2;

    // If using clamped points FF we are using the HOME FIELD
    nearTrench =
        !trenchOverride
            && ((Point.CLAMPED_POINTS_FEATURE_FLAG.getAsBoolean()
                    ? FieldUtil.inHomeFieldTrench(robotPose.getTranslation())
                    : FieldUtil.inTrench(robotPose.getTranslation()))
                || SwerveAssist.ableToTrenchAssist(robotPose, speeds));

    // The tower preset is only used when the driver explicitly requests it. Otherwise keep aiming
    // from the estimated pose so a blocked shot is ready as soon as localization recovers.
    boolean useTowerFallback =
        getState() == RobotState.PREPARE_FALLBACK_SCORE || getState() == RobotState.FALLBACK_SCORE;
    Pose2d robotPoseUsedForScoring =
        useTowerFallback
            ? new Pose2d(
                FieldUtil.getTowerFallbackScorePoint().getTranslation(), robotPose.getRotation())
            : robotPose;
    DogLog.log("RobotManager/ScorePoint", robotPoseUsedForScoring);

    if (useTowerFallback) {
      // The tower preset is a stationary shot. Drive velocity must not move its target or
      // introduce feedforward, including when the delayed SOTM request becomes active.
      scoringParameters =
          AimParameterUtil.getStaticScoringParameters(
              robotPoseUsedForScoring, new ChassisSpeeds(), new ChassisSpeeds());
    } else {
      scoringParameters =
          getState() == RobotState.WARMUP_SCORE || !swerve.driverWantsSotm()
              ? AimParameterUtil.getStaticScoringParameters(
                  robotPoseUsedForScoring, speeds, swerve.getAppliedFieldRelativeSpeeds())
              : AimParameterUtil.getScoringParameters(
                  robotPoseUsedForScoring, speeds, swerve.getAppliedFieldRelativeSpeeds());
    }

    feedingParameters =
        getState() == RobotState.WARMUP_FEED || !swerve.driverWantsSotm()
            ? AimParameterUtil.getStaticFeedingParameters(
                feedLocation, robotPose, speeds, swerve.getAppliedFieldRelativeSpeeds())
            : AimParameterUtil.getFeedingParameters(
                feedLocation, robotPose, speeds, swerve.getAppliedFieldRelativeSpeeds());

    hubActivity.updateShooterScoringTOF(shooter.getScoreTimeOfFlight(scoringParameters.distance()));

    fallbackFeedingParameters =
        AimParameterUtil.getFallbackFeedingParameters(robotPose.getRotation());
    isInAllianceZone =
        !isLocalizationHealthy()
            ? hubActivity.getTOFBasedHubActive()
            : FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation());

    isInSafeFeedingLocation =
        !isLocalizationHealthy()
            || !FieldUtil.isFeedPathObstructed(
                TurretCalculator.getTurretPose(robotPose).getTranslation(),
                feedLocation.getTranslation());
    isScorePathObstructed =
        isLocalizationHealthy()
            && FieldUtil.isScorePathObstructed(
                TurretCalculator.getTurretPose(robotPose).getTranslation());

    shooter.updateHopperState(hopperManager.isFull());
    DogLog.log("RobotManager/Feeding/IsInSafeFeedingLocation", isInSafeFeedingLocation);
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    if (!isLocalizationHealthy()) {
      switch (currentState) {
        case PREPARE_FEED, FEED -> {
          return RobotState.PREPARE_FALLBACK_FEED;
        }
        default -> {}
      }
    }
    return switch (currentState) {
      // No auto transitions for these states
      case UNJAM, FORCE_SCORE, WARMUP_FEED, WARMUP_SCORE, IDLE -> currentState;
      case PREPARE_FORCE_SCORE -> {
        // Tuning mode intentionally bypasses swerve motion gates; wait for mechanisms only.
        if (turret.atGoal(TurretState.SCORE, scoringParameters)
            && shooter.atGoalDebounced(ShooterState.PREPARE_SCORE)
            && shooterHood.atGoal(ShooterHoodState.SCORING)) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case PREPARE_FALLBACK_SCORE, FALLBACK_SCORE -> {
        // Once shooting, validate the active request with its shooting RPM tolerance.
        var expectedShooterState =
            currentState == RobotState.FALLBACK_SCORE
                ? ShooterState.SCORE
                : ShooterState.PREPARE_SCORE;
        logFallbackScoringTransition(expectedShooterState);
        // In pit functionality, we don't care about anything other than raw mechanism states.
        if (DSOptions.PIT_FUNCTIONALITY.getAsBoolean()) {
          yield shooter.atGoalDebounced(expectedShooterState)
                  && shooterHood.atGoal(ShooterHoodState.SCORING)
              ? RobotState.FALLBACK_SCORE
              : RobotState.PREPARE_FALLBACK_SCORE;
        }

        yield turret.atGoal(TurretState.SCORE, scoringParameters)
                && shooter.atGoalDebounced(expectedShooterState)
                && shooterHood.atGoal(ShooterHoodState.SCORING)
            ? RobotState.FALLBACK_SCORE
            : RobotState.PREPARE_FALLBACK_SCORE;
      }
      case PREPARE_FALLBACK_FEED, FALLBACK_FEED -> {
        // Fallback feeding uses fixed aiming parameters and intentionally allows driving.
        var expectedShooterState =
            currentState == RobotState.FALLBACK_FEED
                ? ShooterState.FEED
                : ShooterState.PREPARE_FEED;
        yield turret.atGoal(TurretState.FEED, fallbackFeedingParameters)
                && shooter.atGoalDebounced(expectedShooterState)
                && shooterHood.atGoal(ShooterHoodState.FEEDING)
                && isInSafeFeedingLocation
                && !nearTrench
            ? RobotState.FALLBACK_FEED
            : RobotState.PREPARE_FALLBACK_FEED;
      }
      case PREPARE_SCORE -> {
        logScoringTransition(currentState);

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if (turret.atGoal(TurretState.SCORE, scoringParameters)
            && !swerve.isMovingBeyondSafeSpeed()
            && !swerve.driverStillDecidingSotm()
            && localization.imu.accelerationLowEnoughToShoot()
            && shooter.atGoalDebounced(ShooterState.PREPARE_SCORE)
            && shooterHood.atGoal(ShooterHoodState.SCORING)
            && isLocalizationHealthy()
            && (DriverStation.isAutonomous() || localization.isTrustworthy())
            && hubActivity.getTOFBasedHubActive()
            && !isScorePathObstructed
            && !nearTrench) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case SCORE -> {
        logScoringTransition(currentState);

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if ((!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
                || (turret.atGoal(TurretState.SCORE, scoringParameters)
                    && !swerve.isMovingBeyondSafeSpeed()
                    && !swerve.driverStillDecidingSotm()
                    && localization.imu.accelerationLowEnoughToShoot()
                    && shooter.atGoalDebounced(ShooterState.SCORE)
                    && isLocalizationHealthy()
                    && (DriverStation.isAutonomous() || localization.isTrustworthy())
                    && shooterHood.atGoal(ShooterHoodState.SCORING)
                    && hubActivity.getTOFBasedHubActive()))
            && !isScorePathObstructed
            && !nearTrench) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case PREPARE_FEED -> {
        logFeedTransition(currentState);

        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        if (turret.atGoal(TurretState.FEED, feedingParameters)
            && !swerve.isMovingBeyondSafeSpeed()
            && shooter.atGoalDebounced(ShooterState.PREPARE_FEED)
            && isInSafeFeedingLocation
            && shooterHood.atGoal(ShooterHoodState.FEEDING)
            && isLocalizationHealthy()
            && !nearTrench) {

          yield RobotState.FEED;
        } else {
          yield currentState;
        }
      }
      case FEED -> {
        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        logFeedTransition(currentState);
        if (!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
            || (shooter.atGoalDebounced(ShooterState.FEED)
                && !swerve.isMovingBeyondSafeSpeed()
                && isInSafeFeedingLocation
                && turret.atGoal(TurretState.FEED, feedingParameters)
                && shooterHood.atGoal(ShooterHoodState.FEEDING)
                && isLocalizationHealthy()
                && !nearTrench)) {

          yield currentState;
        } else {

          yield RobotState.PREPARE_FEED;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE -> {
        // Intentionally exclude powerManager.idleRequest() here to allow temporary
        // overrides (e.g. prioritizeIntakeRequest, firstAutoSegmentRequest, turboRequest)
        // to persist without being immediately reset every 20ms cycle.
        // Returning to IDLE from another state resets powerManager via afterTransition.
        shooter.idleRequest();
        shooterHood.idleRequest();
        smartTurretIdleRequest();
        swerve.normalDriveRequest();
        hopperManager.idleRequest();
      }

      case PREPARE_SCORE -> {
        shooter.prepareScoreRequest(scoringParameters.distance());
        smartHoodPrepareScoreRequest();
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
        hopperManager.idleRequest();
        // isHubActive always logged
      }
      case SCORE -> {
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        smartScoringPowerManagerRequest();
      }
      case FORCE_SCORE -> {
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        powerManager.scoringRequest();
      }
      case PREPARE_FORCE_SCORE -> {
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        powerManager.scoringRequest();
        hopperManager.idleRequest();
      }
      case PREPARE_FEED -> {
        shooter.prepareFeedRequest(feedingParameters.distance());
        smartHoodPrepareFeedRequest();
        turret.feedRequest(feedingParameters);
        swerve.feedRequest();
        hopperManager.idleRequest();
        smartFeedingPowerManagerRequest();
      }
      case FEED -> {
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(feedingParameters);
        swerve.feedRequest();
        hopperManager.feedRequest();
        smartFeedingPowerManagerRequest();
      }

      // Fallback states
      case PREPARE_FALLBACK_SCORE -> {
        hopperManager.idleRequest();
        // Automatically update scoring parameters with preset pose
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        smartScoringPowerManagerRequest();
      }
      case FALLBACK_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(scoringParameters);
        swerve.scoreRequest();
        hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        smartScoringPowerManagerRequest();
      }
      case PREPARE_FALLBACK_FEED -> {
        hopperManager.idleRequest();
        shooter.prepareFeedRequest(fallbackFeedingParameters.distance());
        smartHoodPrepareFallbackFeedRequest();
        turret.feedRequest(fallbackFeedingParameters);
        swerve.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case FALLBACK_FEED -> {
        shooter.feedRequest(fallbackFeedingParameters.distance());
        shooterHood.feedRequest(fallbackFeedingParameters.distance());
        turret.feedRequest(fallbackFeedingParameters);
        swerve.feedRequest();
        hopperManager.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case WARMUP_SCORE -> {
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.idleRequest();
        turret.scoreRequest(scoringParameters);
        swerve.warmupScoreRequest();
        hopperManager.idleRequest();
        smartScoringPowerManagerRequest();
      }
      case WARMUP_FEED -> {
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.idleRequest();
        turret.feedRequest(feedingParameters);
        swerve.warmupFeedRequest();
        hopperManager.idleRequest();
        smartFeedingPowerManagerRequest();
      }
      case UNJAM -> {
        hopperManager.unjamRequest();
        shooter.scoreRequest(3);
        shooterHood.scoreRequest(3);
        smartTurretIdleRequest();
        swerve.normalDriveRequest();
        powerManager.idleRequest();
      }
      default -> {}
    }

    DogLog.log("RobotManager/Feeding/FeedLocation", feedLocation);
    DogLog.log("RobotManager/Feeding/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/Scoring/ScoringParameters", scoringParameters);

    MechanismVisualizer.log(
        getState(),
        turret.getAngle(),
        shooterHood.getAngle(),
        hopperManager.deploy.getAngleDegrees());
  }
}
