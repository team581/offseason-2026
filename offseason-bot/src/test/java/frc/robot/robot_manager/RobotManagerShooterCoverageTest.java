package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanSubscriber;
import frc.robot.Robot;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.shooter.ShooterConfig;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.shooter_hood.ShooterHoodState;
import frc.robot.swerve.SwerveState;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.vision.TrustFactor;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class RobotManagerShooterCoverageTest {
  private static Robot robot;

  private static ShooterState expectedShooterState(RobotState state) {
    return switch (state) {
      case IDLE -> ShooterState.IDLE;
      case WARMUP_SCORE, PREPARE_SCORE, PREPARE_FORCE_SCORE, PREPARE_FALLBACK_SCORE ->
          ShooterState.PREPARE_SCORE;
      case SCORE, FORCE_SCORE, FALLBACK_SCORE, UNJAM -> ShooterState.SCORE;
      case WARMUP_FEED, PREPARE_FEED, PREPARE_FALLBACK_FEED -> ShooterState.PREPARE_FEED;
      case FEED, FALLBACK_FEED -> ShooterState.FEED;
    };
  }

  private static Object get(Object target, String name) throws Exception {
    var field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.get(target);
  }

  private static void set(Object target, String name, Object value) throws Exception {
    var field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    field.set(target, value);
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    robot = new Robot();
  }

  @EnumSource(RobotState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceEveryStaleDriveRequest(RobotState state) throws Exception {
    var manager = robot.robotManager;
    var swerve = manager.swerve;
    var expected =
        switch (state) {
          case IDLE, UNJAM -> SwerveState.MANUAL;
          case WARMUP_SCORE -> SwerveState.WARMUP_SCORE;
          case WARMUP_FEED -> SwerveState.WARMUP_FEED;
          case PREPARE_SCORE,
              SCORE,
              PREPARE_FORCE_SCORE,
              FORCE_SCORE,
              PREPARE_FALLBACK_SCORE,
              FALLBACK_SCORE ->
              SwerveState.SCORE;
          case PREPARE_FEED, FEED, PREPARE_FALLBACK_FEED, FALLBACK_FEED -> SwerveState.FEED;
        };
    for (var source : DriveSourceType.values()) {
      swerve.setDriveSourceType(source);
      for (boolean entry : new boolean[] {true, false}) {
        for (var stale : SwerveState.values()) {
          switch (stale) {
            case MANUAL -> swerve.normalDriveRequest();
            case WARMUP_SCORE -> swerve.warmupScoreRequest();
            case SCORE -> swerve.scoreRequest();
            case WARMUP_FEED -> swerve.warmupFeedRequest();
            case FEED -> swerve.feedRequest();
            case CLIMB_ASSIST -> swerve.climbAssistDriveRequest();
          }
          if (entry) {
            manager.afterTransition(state);
          } else {
            manager.whileInState(state);
          }
          assertThat(swerve.getState())
              .as("%s entry=%s stale=%s source=%s", state, entry, stale, source)
              .isEqualTo(expected);
          // Manager requests select behavior; autos retain control of the drive source.
          assertThat(((DriveSource) get(swerve, "driveSource")).getDriveSourceType())
              .isEqualTo(source);
        }
      }
    }
    swerve.setDriveSourceType(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
    manager.afterTransition(RobotState.IDLE);
  }

  @EnumSource(RobotState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceStaleRequestsAndRefreshDistances(RobotState state) throws Exception {
    var manager = robot.robotManager;
    var shooter = manager.shooter;
    var expected = expectedShooterState(state);
    // Exercise each callback independently so entry cannot hide a missing held-state request.
    for (boolean entry : new boolean[] {true, false}) {
      for (ShooterState stale : ShooterState.values()) {
        shooter.idleRequest();
        switch (stale) {
          case IDLE -> {}
          case PREPARE_SCORE -> shooter.prepareScoreRequest(1.0);
          case SCORE -> shooter.scoreRequest(1.0);
          case PREPARE_FEED -> shooter.prepareFeedRequest(1.0);
          case FEED -> shooter.feedRequest(1.0);
        }
        set(manager, "scoringParameters", new AimingParameters(-180, 2.79, 5, 0, -180));
        set(manager, "feedingParameters", new AimingParameters(0, 8.71, 5, 0, 0));
        set(manager, "fallbackFeedingParameters", new AimingParameters(-180, 8.0, 5, 0, -180));
        if (entry) {
          manager.afterTransition(state);
        } else {
          manager.whileInState(state);
        }
        assertThat(shooter.getState())
            .as("%s entry=%s stale=%s", state, entry, stale)
            .isEqualTo(expected);
        switch (expected) {
          case IDLE -> {}
          case PREPARE_SCORE, SCORE ->
              assertThat(get(shooter, "scoreDistance"))
                  .isEqualTo(state == RobotState.UNJAM ? 3.0 : 2.79);
          case PREPARE_FEED, FEED ->
              assertThat(get(shooter, "feedDistance"))
                  .isEqualTo(
                      state == RobotState.PREPARE_FALLBACK_FEED || state == RobotState.FALLBACK_FEED
                          ? 8.0
                          : 8.71);
        }
      }
    }
    manager.idleRequest();
  }

  @EnumSource(
      value = RobotState.class,
      names = {
        "FALLBACK_FEED",
        "FALLBACK_SCORE",
        "PREPARE_FALLBACK_FEED",
        "PREPARE_FALLBACK_SCORE"
      })
  @ParameterizedTest
  void fallbackTransitionsRequireReadinessForTheirCurrentShooterState(RobotState state)
      throws Exception {
    var manager = robot.robotManager;
    var shooter = manager.shooter;
    var hood = (ShooterHood) get(manager, "shooterHood");
    var expected = expectedShooterState(state);
    set(manager, "scoringParameters", new AimingParameters(-180, 2.79, 5, 0, -180));
    set(manager, "fallbackFeedingParameters", new AimingParameters(-180, 8.0, 5, 0, -180));
    var pit = (BooleanEntry) DSOptions.PIT_FUNCTIONALITY;
    boolean previousPit = pit.get();
    try {
      for (boolean pitMode : new boolean[] {false, true}) {
        pit.set(pitMode);
        set(manager, "nearTrench", false);
        set(manager, "isMoving", false);
        set(manager, "isScorePathObstructed", false);
        set(manager, "isInSafeFeedingLocation", true);
        // Homing completion normally happens in the hood simulation periodic.
        hood.homingRequest();
        hood.simulationPeriodic();
        manager.afterTransition(state);
        set(robot.turret, "setpoint", -180.0);
        set(robot.turret, "currentAngle", -180.0);
        var goalAngle = ShooterHood.class.getDeclaredMethod("getGoalAngle");
        goalAngle.setAccessible(true);
        set(hood, "currentAngle", goalAngle.invoke(hood));
        assertThat(
                hood.atGoal(
                    state.isScoring() ? ShooterHoodState.SCORING : ShooterHoodState.FEEDING))
            .isTrue();
        set(shooter, "atGoalDebounced", true);
        assertThat(manager.getNextState(state))
            .isEqualTo(state.isScoring() ? RobotState.FALLBACK_SCORE : RobotState.FALLBACK_FEED);
        set(shooter, "atGoalDebounced", false);
        assertThat(manager.getNextState(state))
            .isEqualTo(
                state.isScoring()
                    ? RobotState.PREPARE_FALLBACK_SCORE
                    : RobotState.PREPARE_FALLBACK_FEED);
        // A true cached readiness bit from a different request must never start feeding balls.
        if (expected == ShooterState.SCORE || expected == ShooterState.PREPARE_SCORE) {
          shooter.feedRequest(8.0);
        } else {
          shooter.scoreRequest(2.79);
        }
        set(shooter, "atGoalDebounced", true);
        assertThat(manager.getNextState(state))
            .isEqualTo(
                state.isScoring()
                    ? RobotState.PREPARE_FALLBACK_SCORE
                    : RobotState.PREPARE_FALLBACK_FEED);
      }
    } finally {
      pit.set(previousPit);
      shooter.idleRequest();
    }
  }

  @EnumSource(
      value = RobotState.class,
      names = {"PREPARE_SCORE", "SCORE", "PREPARE_FEED", "FEED"})
  @ParameterizedTest
  void normalShotsRequireCurrentShooterReadinessAndSafeDriveSpeed(RobotState state)
      throws Exception {
    var manager = robot.robotManager;
    var shooter = manager.shooter;
    var swerve = manager.swerve;
    var hood = (ShooterHood) get(manager, "shooterHood");
    var health = get(manager, "health");
    var pit = (BooleanEntry) DSOptions.PIT_FUNCTIONALITY;
    var vision = (BooleanEntry) DSOptions.USE_VISION;
    var cancelSubscriber = (BooleanSubscriber) FeatureFlags.CANCEL_IN_PROGRESS_SHOT;
    var cancel = cancelSubscriber.getTopic().getEntry(false);
    boolean previousPit = pit.get();
    boolean previousVision = vision.get();
    boolean previousCancel = cancelSubscriber.get();
    var previousSpeeds = get(swerve, "fieldRelativeSpeeds");
    var previousLimit = get(swerve, "currentMaxLinearRate");
    var previousHealthy = get(health, "localizationHealthy");
    try {
      pit.set(false);
      vision.set(true);
      cancel.set(true);
      set(health, "localizationHealthy", true);
      set(manager, "isInAllianceZone", state.isScoring());
      set(manager, "nearTrench", false);
      set(manager, "isScorePathObstructed", false);
      set(manager, "isInSafeFeedingLocation", true);
      set(manager, "scoringParameters", new AimingParameters(-180, 2.79, 5, 0, -180));
      set(manager, "feedingParameters", new AimingParameters(-180, 8.0, 5, 0, -180));
      ((TrustFactor) get(manager.localization, "trustFactor")).reset(0, true);
      set(manager.localization.imu, "accel", 0.0);
      set(swerve, "driverStillDecidingSotm", false);
      set(swerve, "currentMaxLinearRate", 4.0);
      set(swerve, "fieldRelativeSpeeds", new ChassisSpeeds());
      hood.homingRequest();
      hood.simulationPeriodic();
      manager.afterTransition(state);
      set(robot.turret, "setpoint", -180.0);
      set(robot.turret, "currentAngle", -180.0);
      var goalAngle = ShooterHood.class.getDeclaredMethod("getGoalAngle");
      goalAngle.setAccessible(true);
      set(hood, "currentAngle", goalAngle.invoke(hood));
      set(shooter, "atGoalDebounced", true);
      var active = state.isScoring() ? RobotState.SCORE : RobotState.FEED;
      var preparing = state.isScoring() ? RobotState.PREPARE_SCORE : RobotState.PREPARE_FEED;
      assertThat(manager.getNextState(state)).isEqualTo(active);
      set(swerve, "fieldRelativeSpeeds", new ChassisSpeeds(-5, 0, 0));
      assertThat(manager.getNextState(state)).isEqualTo(preparing);
      set(swerve, "fieldRelativeSpeeds", new ChassisSpeeds());
      if (state.isScoring()) {
        set(swerve, "driverStillDecidingSotm", true);
        assertThat(manager.getNextState(state)).isEqualTo(preparing);
        set(swerve, "driverStillDecidingSotm", false);
        set(manager.localization.imu, "accel", 10.0);
        assertThat(manager.getNextState(state)).isEqualTo(preparing);
        set(manager.localization.imu, "accel", 0.0);
      }
      set(shooter, "atGoalDebounced", false);
      assertThat(manager.getNextState(state)).isEqualTo(preparing);
      // Readiness for a different shooter request cannot satisfy this manager state.
      if (state.isScoring()) {
        shooter.feedRequest(8.0);
      } else {
        shooter.scoreRequest(2.79);
      }
      set(shooter, "atGoalDebounced", true);
      assertThat(manager.getNextState(state)).isEqualTo(preparing);
    } finally {
      pit.set(previousPit);
      vision.set(previousVision);
      cancel.set(previousCancel);
      cancel.close();
      set(health, "localizationHealthy", previousHealthy);
      set(swerve, "fieldRelativeSpeeds", previousSpeeds);
      set(swerve, "currentMaxLinearRate", previousLimit);
      manager.afterTransition(RobotState.IDLE);
    }
  }

  @EnumSource(
      value = ShooterState.class,
      names = {"FEED", "SCORE"})
  @ParameterizedTest
  void prepareRequestsReplaceActiveRequestsAndInvalidateReadiness(ShooterState active)
      throws Exception {
    var shooter = robot.robotManager.shooter;
    if (active == ShooterState.FEED) {
      shooter.feedRequest(8.0);
      set(shooter, "atGoalDebounced", true);
      shooter.prepareFeedRequest(9.0);
      assertThat(shooter.getState()).isEqualTo(ShooterState.PREPARE_FEED);
      assertThat(shooter.atGoalDebounced(ShooterState.PREPARE_FEED)).isFalse();
    } else {
      shooter.scoreRequest(2.79);
      set(shooter, "atGoalDebounced", true);
      shooter.prepareScoreRequest(3.46);
      assertThat(shooter.getState()).isEqualTo(ShooterState.PREPARE_SCORE);
      assertThat(shooter.atGoalDebounced(ShooterState.PREPARE_SCORE)).isFalse();
    }
  }

  @Test
  void rpmLimitsAndPitOverrideApplyToBothSetpoints() throws Exception {
    var shooter = robot.robotManager.shooter;
    var pit = (BooleanEntry) DSOptions.PIT_FUNCTIONALITY;
    var regressionSubscriber = (BooleanSubscriber) FeatureFlags.REGRESSION_MODEL;
    var regression = regressionSubscriber.getTopic().getEntry(false);
    boolean previousPit = pit.get();
    boolean previousRegression = regressionSubscriber.get();
    try {
      regression.set(true);
      assertThat(FeatureFlags.REGRESSION_MODEL.getAsBoolean()).isTrue();
      pit.set(false);
      shooter.scoreRequest(1000.0);
      shooter.feedRequest(1000.0);
      shooter.beforePeriodic();
      assertThat(get(shooter, "shootingRpm")).isEqualTo(ShooterConfig.MAX_SAFE_RPM);
      // The feeding regression extrapolates below zero at this extreme distance.
      assertThat(get(shooter, "feedingRpm")).isEqualTo(0.0);
      pit.set(true);
      shooter.beforePeriodic();
      assertThat(get(shooter, "shootingRpm")).isEqualTo(ShooterConfig.PIT_FUNCTIONALITY_RPM);
      assertThat(get(shooter, "feedingRpm")).isEqualTo(ShooterConfig.PIT_FUNCTIONALITY_RPM);
    } finally {
      pit.set(previousPit);
      regression.set(previousRegression);
      regression.close();
      shooter.idleRequest();
    }
  }

  @Test
  void safeSpeedUsesMeasuredTranslationInEveryDirection() throws Exception {
    var swerve = robot.robotManager.swerve;
    var previousSpeeds = get(swerve, "fieldRelativeSpeeds");
    var previousLimit = get(swerve, "currentMaxLinearRate");
    try {
      set(swerve, "currentMaxLinearRate", 4.0);
      for (var speeds :
          new ChassisSpeeds[] {
            new ChassisSpeeds(4.01, 0, 0),
            new ChassisSpeeds(-4.01, 0, 0),
            new ChassisSpeeds(0, -4.01, 0),
            new ChassisSpeeds(3, 3, 0)
          }) {
        set(swerve, "fieldRelativeSpeeds", speeds);
        assertThat(swerve.isMovingBeyondSafeSpeed()).isTrue();
      }
      set(swerve, "fieldRelativeSpeeds", new ChassisSpeeds(4, 0, 0));
      assertThat(swerve.isMovingBeyondSafeSpeed()).isFalse();
      set(swerve, "fieldRelativeSpeeds", new ChassisSpeeds());
      assertThat(swerve.isMovingBeyondSafeSpeed()).isFalse();
    } finally {
      set(swerve, "fieldRelativeSpeeds", previousSpeeds);
      set(swerve, "currentMaxLinearRate", previousLimit);
    }
  }
}
