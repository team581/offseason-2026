package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.PoseErrorTolerance;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.LeftBumpDepotAutoState;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.BumpCrossingTracker;
import frc.robot.robot_manager.RobotManager;

public class LeftBumpDepotAuto extends BaseImperativeAuto<LeftBumpDepotAutoState> {

  public enum Markers {
    PRIORITIZE_INTAKE,
    START_SHOOT_RQ,
    READY_TO_CROSS_BUMP
  }

  private static final double COLLISION_X_OFFSET = -0.1;

  private static final double BUMP_OFFSET = -0.13;

  static final double SHOOT_X = 13.9;
  static final double RETURN_BUMP_LANDING_X = 13.25;
  static final double DEPOT_DRIVER_STATION_OFFSET = Units.inchesToMeters(5.0);
  // Leave space for the rotated bumpers below the longitudinal field centerline.
  static final double SWEEP_MAX_RED_Y = 3.3;

  // Stay outside the depot while shooting and below the tower with the rotated bumper envelope.
  static final Point DEPOT_APPROACH =
      Point.ofRed(new Pose2d(15.2, 3.0, Rotation2d.fromDegrees(-75)));
  static final Point DEPOT_EXIT =
      Point.ofRed(
          new Pose2d(15.85 + DEPOT_DRIVER_STATION_OFFSET, 1.0, Rotation2d.fromDegrees(-75)));
  static final Point DEPOT_FRONT = Point.ofRed(new Pose2d(15.15, 1.3, Rotation2d.fromDegrees(-75)));
  static final Point FINAL_SHOOT_GOAL =
      Point.ofRed(new Pose2d(14.9, 1.8, Rotation2d.fromDegrees(-75)));

  static final double FIRST_SHOOT_MIN_SECONDS = 5.0;

  private Point alignmentPoint = DEPOT_APPROACH;

  private final AutoSegment crossBumpOut =
      Trailblazer.segment(
              AutoPoint.ofRed(
                  new Pose2d(10.4, FieldUtil.RED_DEPOT_BUMP_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(2.0, 2.0)
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment alignForBumpOut =
      Trailblazer.segment(
              AutoPoint.of(() -> alignmentPoint)
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 5)),
              AutoPoint.ofRed(
                  new Pose2d(13.25, FieldUtil.RED_DEPOT_BUMP_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(2.0, 2.0)
          .withAngularConstraints(4.0, 3.0)
          .untilFinished(new PoseErrorTolerance(0.1, 5));

  private BumpCrossingTracker bumpCrossingTracker;

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeFirstCycle =
      Trailblazer.segment(
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(9.200, 1.850, Rotation2d.fromDegrees(-155)))))
                  .withMarker(Markers.PRIORITIZE_INTAKE)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, 1.850, Rotation2d.fromDegrees(105)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, 2.500, Rotation2d.kCCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, SWEEP_MAX_RED_Y, Rotation2d.kCCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(8.744, SWEEP_MAX_RED_Y, Rotation2d.fromDegrees(35)))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(9.556, 3.050, Rotation2d.fromDegrees(-25)))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 55)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.15,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withAngularConstraints(2.5, 2.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 35)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.6,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withMarker(Markers.READY_TO_CROSS_BUMP)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment crossBumpToShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment driveToTower =
      Trailblazer.segment(AutoPoint.of(DEPOT_APPROACH))
          .withLinearConstraints(2.0, 3.0)
          .withAngularConstraints(2.0, 3.0)
          .untilFinished(new PoseErrorTolerance(0.08, 3));

  private final AutoSegment collectDepot =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          15.7 + DEPOT_DRIVER_STATION_OFFSET, 3.0, Rotation2d.fromDegrees(-75)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.08, 3)),
              AutoPoint.of(DEPOT_EXIT))
          .withLinearConstraints(1.0, 1.0)
          .withAngularConstraints(0.8, 1.2)
          .untilFinished(new PoseErrorTolerance(0.08, 3));

  private final AutoSegment driveToDepotFront =
      Trailblazer.segment(AutoPoint.of(DEPOT_FRONT))
          .withLinearConstraints(2.0, 3.0)
          .withAngularConstraints(0.8, 1.2)
          .untilFinished(new PoseErrorTolerance(0.08, 3));

  private final AutoSegment shootBackToHub =
      Trailblazer.segment(AutoPoint.of(FINAL_SHOOT_GOAL))
          .withLinearConstraints(0.2, 0.3)
          .withAngularConstraints(0.4, 0.8)
          .untilFinished(new PoseErrorTolerance(0.08, 3));

  private AutoSegment stuckOnBall =
      StuckOnBallRecovery.getRecoverySegment(
          () -> robotManager.localization.getPose(),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getPitch()),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getRoll()));

  private LeftBumpDepotAutoState storedStuckOnBallState = LeftBumpDepotAutoState.INTAKE_FIRST_CYCLE;

  private AutoSegment storedStuckOnBallAutoSegment = intakeFirstCycle;
  private int storedStuckOnBallIndex = 0;
  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;

  public LeftBumpDepotAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(LeftBumpDepotAutoState.PREPARE_BUMP_OUT_1, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return BumpAutoStart.atHubCorner(true);
  }

  @Override
  public boolean shouldRun() {
    return DriverStation.isAutonomousEnabled();
  }

  private Point getCollisionPoint(Point point) {
    if (collisionEverDetected) {
      return Point.ofRed(
          new Pose2d(
              point.redPose().getX() + COLLISION_X_OFFSET,
              point.redPose().getY(),
              point.redPose().getRotation()));
    } else {
      return point;
    }
  }

  @Override
  protected void afterTransition(LeftBumpDepotAutoState newState) {
    switch (newState) {
      case PREPARE_BUMP_OUT_1 -> {
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
        var pose = robotManager.localization.getPose();
        var redPose = FmsUtil.isRedAlliance() ? pose : FieldUtil.pathflip(pose);
        alignmentPoint = Point.ofRed(new Pose2d(redPose.getTranslation(), Rotation2d.k180deg));
        trailblazer.setActiveSegment(alignForBumpOut);
        robotManager.idleRequest();
      }
      case CROSS_BUMP_OUT -> {
        trailblazer.setActiveSegment(crossBumpOut);
        bumpCrossingTracker.bumpCrossRequest(
            Point.ofRed(
                new Pose2d(10.6, FieldUtil.RED_DEPOT_BUMP_CENTER.getY(), Rotation2d.k180deg)),
            FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);
      }
      case INTAKE_FIRST_CYCLE -> {
        storedStuckOnBallAutoSegment = intakeFirstCycle;
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootOne;
        bumpCrossingTracker.bumpCrossRequest(
            Point.ofRed(
                new Pose2d(
                    RETURN_BUMP_LANDING_X,
                    FieldUtil.RED_DEPOT_BUMP_CENTER.getY() - BUMP_OFFSET,
                    Rotation2d.kZero)),
            FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
      }
      case DRIVE_TO_TOWER -> {
        storedStuckOnBallAutoSegment = driveToTower;
      }
      case SHOOT_1 -> {
        trailblazer.clearActiveSegment();
      }
      case COLLECT_DEPOT -> {
        robotManager.idleRequest();
        robotManager.intakeRequest();
        robotManager.powerManager.prioritizeIntakeRequest();
      }
      case DRIVE_TO_DEPOT_FRONT -> {
        robotManager.cancelIntakeRequest();
        robotManager.idleRequest();
        robotManager.powerManager.idleRequest();
      }
      case SHOOT_BACK_TO_HUB -> {
        robotManager.cancelIntakeRequest();
        robotManager.powerManager.idleRequest();
      }
      case DONE -> {
        robotManager.cancelIntakeRequest();
        robotManager.idleRequest();
        trailblazer.clearActiveSegment();
      }
      case STUCK_ON_BALL_RECOVERY -> {}
    }
  }

  @Override
  protected void beforeTransition(
      LeftBumpDepotAutoState oldState, LeftBumpDepotAutoState newState) {

    if (newState == LeftBumpDepotAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
      DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
      DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);
    }

    if (oldState == LeftBumpDepotAutoState.STUCK_ON_BALL_RECOVERY) {
      trailblazer.setActiveSegment(storedStuckOnBallAutoSegment, storedStuckOnBallIndex);
    }
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    if (RobotBase.isSimulation()) {
      BumpAutoSimulation.update(robotManager.localization.imu, robotManager.localization.getPose());
    }
    if (!collisionEverDetected
        && getState() == LeftBumpDepotAutoState.INTAKE_FIRST_CYCLE
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
      DogLog.log("LeftBumpDepotAuto/CollisionDetected", collisionEverDetected);
    }
  }

  @Override
  protected LeftBumpDepotAutoState getNextState(LeftBumpDepotAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
        || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()) {
      switch (currentState) {
        case INTAKE_FIRST_CYCLE -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return LeftBumpDepotAutoState.STUCK_ON_BALL_RECOVERY;
          }
        }
        default -> {}
      }
    }

    return switch (currentState) {
      case PREPARE_BUMP_OUT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield LeftBumpDepotAutoState.CROSS_BUMP_OUT;
        }
        yield currentState;
      }
      case CROSS_BUMP_OUT -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield LeftBumpDepotAutoState.INTAKE_FIRST_CYCLE;
        }
        yield currentState;
      }
      case STUCK_ON_BALL_RECOVERY -> {
        if (!StuckOnBallRecovery.stuckOnBall(
            robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
          yield storedStuckOnBallState;
        } else {
          yield currentState;
        }
      }
      case INTAKE_FIRST_CYCLE -> {
        if (trailblazer.passedMarker(Markers.READY_TO_CROSS_BUMP)) {
          robotManager.powerManager.idleRequest();
          yield LeftBumpDepotAutoState.CROSS_BUMP_TO_SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        // Drive clear of the bump as soon as the landing is detected.
        if (bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield LeftBumpDepotAutoState.DRIVE_TO_TOWER;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_TOWER -> {
        var speeds = robotManager.swerve.getFieldRelativeSpeeds();
        yield trailblazer.atGoal(robotManager.localization.getPose())
                && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1
                && Math.abs(speeds.omegaRadiansPerSecond) < 0.1
            ? LeftBumpDepotAutoState.SHOOT_1
            : currentState;
      }
      case SHOOT_1 -> {
        yield timeout(FIRST_SHOOT_MIN_SECONDS)
            ? LeftBumpDepotAutoState.COLLECT_DEPOT
            : currentState;
      }
      case COLLECT_DEPOT -> {
        yield trailblazer.atGoal(robotManager.localization.getPose())
            ? LeftBumpDepotAutoState.DRIVE_TO_DEPOT_FRONT
            : currentState;
      }
      case DRIVE_TO_DEPOT_FRONT -> {
        yield trailblazer.atGoal(robotManager.localization.getPose())
            ? LeftBumpDepotAutoState.SHOOT_BACK_TO_HUB
            : currentState;
      }
      // Keep requesting the final shot until autonomous is disabled or ends.
      case SHOOT_BACK_TO_HUB -> currentState;
      case DONE -> LeftBumpDepotAutoState.DONE;
    };
  }

  @Override
  protected void whileInState(LeftBumpDepotAutoState newState) {
    if (getState() != LeftBumpDepotAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallIndex = trailblazer.getCurrentPointIndex();
    }

    switch (newState) {
      case PREPARE_BUMP_OUT_1 -> trailblazer.setActiveSegment(alignForBumpOut);
      case CROSS_BUMP_OUT -> {
        trailblazer.setActiveSegment(crossBumpOut);
        robotManager.intakeRequest();
        robotManager.powerManager.firstAutoSegmentRequest();
      }
      case STUCK_ON_BALL_RECOVERY -> {
        trailblazer.setActiveSegment(stuckOnBall);

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.0)
            && RobotBase.isSimulation()) {
          robotManager.localization.imu.setPitch(0.0);
          robotManager.localization.imu.setRoll(0.0);
        }
      }
      case INTAKE_FIRST_CYCLE -> {
        trailblazer.setActiveSegment(intakeFirstCycle);
        robotManager.intakeRequest();
        if (trailblazer.passedMarker(Markers.PRIORITIZE_INTAKE)) {
          robotManager.powerManager.prioritizeIntakeRequest();
        } else {
          robotManager.powerManager.firstAutoSegmentRequest();
        }

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !firstStuckOnBall) {
          firstStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        trailblazer.setActiveSegment(crossBumpToShootOne);
        robotManager.cancelIntakeRequest();
        // Aim and spin up using the live shooting solution before the bump finishes.
        // WARMUP_SCORE keeps the hopper idle throughout the crossing.
        robotManager.warmupScoreRequest();
      }
      case DRIVE_TO_TOWER -> {
        trailblazer.setActiveSegment(driveToTower);
        robotManager.warmupScoreRequest();
      }
      case SHOOT_1 -> {
        robotManager.prepareScoreRequest();
      }
      case COLLECT_DEPOT -> {
        trailblazer.setActiveSegment(collectDepot);
        robotManager.intakeRequest();
      }
      case DRIVE_TO_DEPOT_FRONT -> {
        trailblazer.setActiveSegment(driveToDepotFront);
        // Keep the hopper idle until the robot is fully in front of the depot.
        robotManager.warmupScoreRequest();
      }
      case SHOOT_BACK_TO_HUB -> {
        trailblazer.setActiveSegment(shootBackToHub);
        robotManager.prepareScoreRequest();
      }
      case DONE -> {}
    }
  }
}
