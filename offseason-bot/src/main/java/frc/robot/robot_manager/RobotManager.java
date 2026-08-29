package frc.robot.robot_manager;

import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import com.team581.swerve.SwerveAssist;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Hardware;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private static final double PRESET_FEED_DISTANCE = 0.0;
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
  private final Trailblazer trailblazer;

  public final ClusterMap clusterMap;

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
      Vision vision,
      XboxController driverController,
      HealthManager health,
      HubActivity hubActivity,
      Trailblazer trailblazer,
      ClusterMap clusterMap,
      Hardware hardware,
      PowerManager powerManager,
      Turret turret) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.hopperManager = hopperManager;
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.vision = vision;
    this.driverController = driverController;
    this.health = health;
    this.hubActivity = hubActivity;
    this.trailblazer = trailblazer;
    this.clusterMap = clusterMap;

    this.hardware = hardware;
    this.powerManager = powerManager;
    this.turret = turret;
  }

  public void cancelIntakeRequest() {
    if (hopperManager != null) hopperManager.setDriverWantsIntake(false);
  }

  public void cancelStowDeployRequest() {
    if (hopperManager != null) hopperManager.setOperatorWantsStow(false);
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

  public void forceShootRequest() {
    if (getState() != RobotState.FORCE_SCORE) {
      setStateFromRequest(RobotState.PREPARE_FORCE_SCORE);
    }
  }

  public void homeDeployInAutoRequest() {
    if (hopperManager != null) hopperManager.deploy.homeInAutoRequest();
  }

  public void homeDeployRequest() {
    if (hopperManager != null) hopperManager.deploy.homingRequest();
  }

  public void homeShooterHoodRequest() {
    shooterHood.homingRequest();
  }

  public void idleRequest() {
    setStateFromRequest(RobotState.IDLE);
  }

  public void intakeAutoRequest() {
    if (hopperManager != null) hopperManager.setDriverWantsIntake(true);
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

  public void setTrenchOverrideRequest(boolean trenchOverride) {
    this.trenchOverride = trenchOverride;
  }

  public void stowDeployRequest() {
    if (hopperManager != null) hopperManager.setOperatorWantsStow(true);
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

  private void logFeedTransition() {
    DogLog.log("RobotManager/Feeding/FeedTransition/NotInAllianceZone", !isInAllianceZone);
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/TurretAtGoal", turret.atGoal(feedingParameters));
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/ShooterAtGoalDebounced", shooter.atGoalDebounced());
    DogLog.log("RobotManager/Feeding/FeedTransition/SafeFeedLocation", isInSafeFeedingLocation);
    DogLog.log("RobotManager/Feeding/FeedTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Feeding/FeedTransition/LocalizationHealthy", health.isLocalizationHealthy());
    DogLog.log("RobotManager/Feeding/ScoreTransition/NotNearTrench", !nearTrench);
    DogLog.log("RobotManager/Feeding/FeedTransition/IsMoving", isMoving);
  }

  private void logScoringTransition() {
    DogLog.log("RobotManager/Scoring/ScoreTransition/InAllianceZone", isInAllianceZone);
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/TurretAtGoal", turret.atGoal(scoringParameters));
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/ShooterAtGoalDebounced", shooter.atGoalDebounced());
    DogLog.log("RobotManager/Scoring/ScoreTransition/ShooterHoodAtGoal", shooterHood.atGoal());
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/LocalizationTrustworthy",
        localization.isTrustworthy());

    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/HubActive", hubActivity.getTOFBasedHubActive());
    DogLog.log("RobotManager/Scoring/ScoreTransition/NotNearTrench", !nearTrench);
    DogLog.log(
        "RobotManager/Scoring/ScoreTransition/SwerveSafeSpeed", !swerve.isMovingBeyondSafeSpeed());
  }

  private void smartFeedingPowerManagerRequest() {
    if (robotPose.getTranslation().getDistance(feedLocation.getTranslation())
        > SLOW_FEEDING_DISTANCE_THRESHOLD.get()) {
      powerManager.feedingFarRequest();
    } else {
      powerManager.feedingRequest();
    }
  }

  private void smartHoodPrepareScoreRequest() {
    // If cameras are offline or we are near a trench, always be idle
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NearTrench");
    } else {
      DogLog.log("RobotManager/Scoring/SmartPrepareScore/HoodStatus", "NotNearTrench");

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

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        // hoppermanager controlled separately
        vision.hubTagsRequest();
        shooter.idleRequest();
        shooterHood.idleRequest();
        swerve.normalDriveRequest();
        powerManager.idleRequest();
      }
      case PREPARE_FORCE_SCORE -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
        powerManager.scoringRequest();
      }
      case FORCE_SCORE -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        swerve.normalDriveRequest();
        powerManager.scoringRequest();
      }
      case PREPARE_FEED -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartFeedingPowerManagerRequest();
      }
      case FEED -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartFeedingPowerManagerRequest();
      }
      case PREPARE_SCORE -> {
        // hoppermanager controlled separately
        vision.hubTagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartScoringPowerManagerRequest();
      }
      case SCORE -> {
        // hoppermanager controlled separately
        vision.hubTagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartScoringPowerManagerRequest();
      }
      case PREPARE_FALLBACK_FEED -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.prepareFeedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        turret.feedRequest(
            fallbackFeedingParameters.goalAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        powerManager.feedingRequest();
      }
      case FALLBACK_FEED -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.feedRequest(PRESET_FEED_DISTANCE);
        shooterHood.feedRequest(PRESET_FEED_DISTANCE);
        turret.feedRequest(
            fallbackFeedingParameters.goalAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartFeedingPowerManagerRequest();
      }
      case PREPARE_FALLBACK_SCORE -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        powerManager.scoringRequest();
      }
      case FALLBACK_SCORE -> {
        // hoppermanager controlled separately
        vision.tagsRequest();
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        powerManager.scoringRequest();
      }
      case UNJAM -> {
        if (hopperManager != null) hopperManager.unjamRequest();
        vision.tagsRequest();
        shooter.scoreRequest(3);
        shooterHood.scoreRequest(3);
        swerve.normalDriveRequest();
        powerManager.idleRequest();
      }
      case WARMUP_SCORE -> {
        vision.hubTagsRequest();
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.idleRequest();
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        powerManager.scoringRequest();
        if (hopperManager != null) hopperManager.idleRequest();
      }
      case WARMUP_FEED -> {
        vision.tagsRequest();
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.idleRequest();
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartFeedingPowerManagerRequest();
        if (hopperManager != null) hopperManager.idleRequest();
      }
    }
  }

  @Override
  protected void collectInputs() {
    hubActivity.updateShooterScoringTOF(shooter.getScoreTimeOfFlight(scoringParameters.distance()));

    robotPose = localization.getPose();
    feedLocation = FeedLocation.closest(robotPose);
    double robotRotation = robotPose.getRotation().getDegrees();
    if (clusterMap != null && hopperManager != null) {
      clusterMap.setDeployFullyExtended(hopperManager.deploy.isFullyExtended());
    }
    vision.setEstimatedPoseAngle(robotRotation);
    var speeds = swerve.getFieldRelativeSpeeds();
    var velocity = MathHelpers.getLinearVelocity(speeds);
    vision.setRobotVelocity(velocity);

    isMoving = velocity > 0.2;

    // If using clamped points FF we are using the HOME FIELD
    nearTrench =
        !trenchOverride
            && ((Point.CLAMPED_POINTS_FEATURE_FLAG.getAsBoolean()
                    ? FieldUtil.inHomeFieldTrench(robotPose.getTranslation())
                    : FieldUtil.inTrench(robotPose.getTranslation()))
                || SwerveAssist.ableToTrenchAssist(robotPose, speeds));

    Pose2d robotPoseUsedForScoring =
        health.isLocalizationHealthy()
            ? robotPose
            : new Pose2d(
                FieldUtil.getFallbackScorePoint().getTranslation(), robotPose.getRotation());
    scoringParameters =
        getState() == RobotState.WARMUP_SCORE || !swerve.driverWantsSotm()
            ? AimParameterUtil.getStaticScoringParameters(robotPoseUsedForScoring, speeds)
            : AimParameterUtil.getScoringParameters(robotPoseUsedForScoring, speeds);

    feedingParameters =
        getState() == RobotState.WARMUP_FEED || !swerve.driverWantsSotm()
            ? AimParameterUtil.getStaticFeedingParameters(feedLocation, robotPose, speeds)
            : AimParameterUtil.getFeedingParameters(feedLocation, robotPose, speeds);

    fallbackFeedingParameters =
        AimParameterUtil.getFallbackFeedingParameters(robotPose.getRotation());
    isInAllianceZone =
        !health.isLocalizationHealthy()
            ? hubActivity.getTOFBasedHubActive()
            : FieldUtil.isRobotPastObstacleTowardAllianceZone(robotPose.getTranslation());

    isInSafeFeedingLocation =
        !health.isLocalizationHealthy()
            || !FieldUtil.isFeedPathObstructed(
                robotPose.getTranslation(), feedLocation.getTranslation());

    if (hopperManager != null) {
      shooter.updateHopperState(hopperManager.tunnel.getAverageCurrent(), hopperManager.isFull());
    } else {
      shooter.updateHopperState(0.0, false);
    }
    DogLog.log("RobotManager/Feeding/IsInSafeFeedingLocation", isInSafeFeedingLocation);
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      // No auto transitions for these states
      case UNJAM, FORCE_SCORE, WARMUP_FEED, WARMUP_SCORE, IDLE -> currentState;
      case PREPARE_FORCE_SCORE -> {
        if (shooter.atGoalDebounced() && shooterHood.atGoal()) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case PREPARE_FALLBACK_SCORE, FALLBACK_SCORE -> {
        // In pit functionality, we don't care about anything other than raw mechanism
        // states
        if (DSOptions.PIT_FUNCTIONALITY.getAsBoolean()) {
          yield shooter.atGoalDebounced() && shooterHood.atGoal()
              ? RobotState.FALLBACK_SCORE
              : RobotState.PREPARE_FALLBACK_SCORE;
        }

        yield !isMoving
                && (turret.atGoal(scoringParameters)
                    && shooter.atGoalDebounced()
                    && shooterHood.atGoal())
            ? RobotState.FALLBACK_SCORE
            : RobotState.PREPARE_FALLBACK_SCORE;
      }
      case PREPARE_FALLBACK_FEED, FALLBACK_FEED ->
          turret.atGoal(fallbackFeedingParameters)
                  && shooter.atGoalDebounced()
                  && shooterHood.atGoal()
              ? RobotState.FALLBACK_FEED
              : RobotState.PREPARE_FALLBACK_FEED;
      case PREPARE_SCORE -> {
        logScoringTransition();

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if (turret.atGoal(scoringParameters)
            && !swerve.isMovingBeyondSafeSpeed()
            && localization.imu.accelerationLowEnoughToShoot()
            && shooter.atGoalDebounced()
            && shooterHood.atGoal()
            && localization.isTrustworthy()
            && hubActivity.getTOFBasedHubActive()
            && !nearTrench) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case SCORE -> {
        logScoringTransition();

        if (!isInAllianceZone) {
          if (DriverStation.isTeleop()) {
            yield RobotState.PREPARE_FEED;
          }
          yield currentState;
        }

        if ((!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
                || (turret.atGoal(scoringParameters)
                    && !swerve.isMovingBeyondSafeSpeed()
                    && localization.imu.accelerationLowEnoughToShoot()
                    && shooter.atGoalDebounced()
                    && localization.isTrustworthy()
                    && shooterHood.atGoal()
                    && hubActivity.getTOFBasedHubActive()))
            && !nearTrench) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case PREPARE_FEED -> {
        logFeedTransition();

        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        if (turret.atGoal(feedingParameters)
            && !swerve.isMovingBeyondSafeSpeed()
            && shooter.atGoalDebounced()
            && isInSafeFeedingLocation
            && shooterHood.atGoal()
            && health.isLocalizationHealthy()) {

          yield RobotState.FEED;
        } else {
          yield currentState;
        }
      }
      case FEED -> {
        if (isInAllianceZone) {
          yield RobotState.PREPARE_SCORE;
        }

        logFeedTransition();
        if (!FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()
            || (shooter.atGoalDebounced()
                && !swerve.isMovingBeyondSafeSpeed()
                && isInSafeFeedingLocation
                && turret.atGoal(feedingParameters)
                && shooterHood.atGoal()
                && health.isLocalizationHealthy())) {

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
        shooterHood.idleRequest();
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.idleRequest();
      }

      case PREPARE_SCORE -> {
        shooter.prepareScoreRequest(scoringParameters.distance());
        smartHoodPrepareScoreRequest();
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartScoringPowerManagerRequest();
        if (hopperManager != null) hopperManager.idleRequest();
        // isHubActive always logged
      }
      case SCORE -> {
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.scoreRequest(hubActivity.shouldBeastMode());
        smartScoringPowerManagerRequest();
      }
      case FORCE_SCORE -> {
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        if (hopperManager != null) hopperManager.scoreRequest(hubActivity.shouldBeastMode());
      }
      case PREPARE_FEED -> {
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.idleRequest();
        smartFeedingPowerManagerRequest();
      }
      case FEED -> {
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.feedRequest();
        smartFeedingPowerManagerRequest();
      }

      // Fallback states
      case PREPARE_FALLBACK_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooter.prepareScoreRequest(scoringParameters.distance());
        if (isMoving) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(scoringParameters.distance());
        }
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
      }
      case FALLBACK_SCORE -> {
        // Automatically update scoring parameters with preset pose
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.scoreRequest(hubActivity.shouldBeastMode());
      }
      case PREPARE_FALLBACK_FEED -> {
        turret.feedRequest(
            fallbackFeedingParameters.goalAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        smartFeedingPowerManagerRequest();
      }
      case FALLBACK_FEED -> {
        turret.feedRequest(
            fallbackFeedingParameters.goalAngle(),
            fallbackFeedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.feedRequest();
        smartFeedingPowerManagerRequest();
      }
      case WARMUP_SCORE -> {
        shooter.prepareScoreRequest(scoringParameters.distance());
        shooterHood.idleRequest();
        turret.scoreRequest(
            scoringParameters.goalAngle(), scoringParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.idleRequest();
      }
      case WARMUP_FEED -> {
        shooter.prepareFeedRequest(feedingParameters.distance());
        shooterHood.idleRequest();
        turret.feedRequest(
            feedingParameters.goalAngle(), feedingParameters.turretFeedForwardRadians());
        swerve.normalDriveRequest();
        if (hopperManager != null) hopperManager.idleRequest();
        smartFeedingPowerManagerRequest();
      }
      default -> {}
    }

    swerve.setUseLooseTolerance(getState().isFeeding() && timeout(1.0));

    DogLog.log("RobotManager/Feeding/FeedLocation", feedLocation);
    DogLog.log("RobotManager/Feeding/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/Scoring/ScoringParameters", scoringParameters);

    MechanismVisualizer.log(
        getState(),
        turret.getAngle(),
        shooterHood.getAngle(),
        hopperManager != null ? hopperManager.deploy.getPosition() : 0.0);
  }
}
