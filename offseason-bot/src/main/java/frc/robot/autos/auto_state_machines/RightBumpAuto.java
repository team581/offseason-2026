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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.BumpAutoState;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.BumpCrossingTracker;
import frc.robot.robot_manager.RobotManager;

public class RightBumpAuto extends BaseImperativeAuto<BumpAutoState> {

  public enum Markers {
    PRIORITIZE_INTAKE,
    START_SHOOT_RQ,
    READY_TO_CROSS_BUMP
  }

  private static final double SHOOTER_WARMUP_DISTANCE_FROM_HUB = 2.171;

  private static final double COLLISION_X_OFFSET = -0.1;

  private static final double BUMP_OFFSET = -0.13;

  static final double SHOOT_X = 13.9;

  private static final double OPPOSITE_HUB_X = 10.7;
  private static final double OPPOSITE_HUB_Y_OFFSET = -0.25;

  private static final double TRENCH_AUTO_CLEAR_WAIT_SECONDS = 1.0;

  private static final double TOWER_CLEAR_X = 15.0;
  private static final double TOWER_CLEAR_Y = 4.77;

  static final Point TOWER_SHOOT_GOAL =
      NormalAutoSotm.shootingGoal(
          new Translation2d(SHOOT_X, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET),
          new Translation2d(TOWER_CLEAR_X, TOWER_CLEAR_Y),
          false);

  private Point alignmentPoint = TOWER_SHOOT_GOAL;

  private final AutoSegment crossBumpOut =
      Trailblazer.segment(
              AutoPoint.ofRed(
                  new Pose2d(10.4, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(2.0, 2.0)
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment alignForBumpOut =
      Trailblazer.segment(
              AutoPoint.of(() -> alignmentPoint)
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 5)),
              AutoPoint.ofRed(
                  new Pose2d(13.25, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.k180deg)))
          .withLinearConstraints(2.0, 2.0)
          .withAngularConstraints(4.0, 3.0)
          .untilFinished(new PoseErrorTolerance(0.1, 5));

  private final AutoSegment driveToOppositeHub =
      Trailblazer.segment(
              AutoPoint.ofRed(
                  new Pose2d(
                      OPPOSITE_HUB_X,
                      FieldUtil.RED_HUB_CENTER.getY() + OPPOSITE_HUB_Y_OFFSET,
                      Rotation2d.k180deg)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(4.0, 4.0)
          .untilFinished(new PoseErrorTolerance(0.15, 10));

  private BumpCrossingTracker bumpCrossingTracker;

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeFirstCycle =
      Trailblazer.segment(
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(9.200, 3.220, Rotation2d.fromDegrees(-155)))))
                  .withMarker(Markers.PRIORITIZE_INTAKE)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, 3.220, Rotation2d.fromDegrees(105)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, 4.445, Rotation2d.kCCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, 5.570, Rotation2d.kCCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.744, 6.220, Rotation2d.fromDegrees(35.0)))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(9.556, 5.920, Rotation2d.fromDegrees(-25.0)))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 55)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.15,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withAngularConstraints(2.5, 2.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 35)),
              AutoPoint.ofRed(
                      new Pose2d(
                          10.6,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          SHOOT_X,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
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
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment driveToTower =
      Trailblazer.segment(AutoPoint.of(TOWER_SHOOT_GOAL))
          .withLinearConstraints(2.0, 3.0)
          .withAngularConstraints(2.0, 3.0)
          .untilFinished(new PoseErrorTolerance(0.08, 3));

  private AutoSegment stuckOnBall =
      StuckOnBallRecovery.getRecoverySegment(
          () -> robotManager.localization.getPose(),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getPitch()),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getRoll()));

  private BumpAutoState storedStuckOnBallState = BumpAutoState.INTAKE_FIRST_CYCLE;
  private AutoSegment storedStuckOnBallAutoSegment = intakeFirstCycle;
  private int storedStuckOnBallIndex = 0;

  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;

  public RightBumpAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(BumpAutoState.PREPARE_BUMP_OUT_1, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return BumpAutoStart.atHubCorner(false);
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
  protected void afterTransition(BumpAutoState newState) {
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
                new Pose2d(10.6, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.k180deg)),
            FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);
      }
      case DRIVE_TO_OPPOSITE_HUB, WAIT_BEHIND_HUB -> {
        storedStuckOnBallAutoSegment = driveToOppositeHub;
      }
      case INTAKE_FIRST_CYCLE -> {
        storedStuckOnBallAutoSegment = intakeFirstCycle;
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootOne;
        bumpCrossingTracker.bumpCrossRequest(
            Point.ofRed(
                new Pose2d(
                    13.25,
                    FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                    Rotation2d.kZero)),
            FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
      }
      case DRIVE_TO_TOWER -> {
        storedStuckOnBallAutoSegment = driveToTower;
      }
      case SHOOT_1 -> {
        trailblazer.clearActiveSegment();
      }
      case DONE -> {
        robotManager.idleRequest();
      }
      case STUCK_ON_BALL_RECOVERY -> {}
    }
  }

  @Override
  protected void beforeTransition(BumpAutoState oldState, BumpAutoState newState) {

    if (newState == BumpAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
      DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
      DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);
    }

    if (oldState == BumpAutoState.STUCK_ON_BALL_RECOVERY) {
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
        && getState() == BumpAutoState.INTAKE_FIRST_CYCLE
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
      DogLog.log("RightBumpAuto/CollisionDetected", collisionEverDetected);
    }
  }

  @Override
  protected BumpAutoState getNextState(BumpAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
        || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()) {
      switch (currentState) {
        case DRIVE_TO_OPPOSITE_HUB, INTAKE_FIRST_CYCLE -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return BumpAutoState.STUCK_ON_BALL_RECOVERY;
          }
        }
        default -> {}
      }
    }

    return switch (currentState) {
      case PREPARE_BUMP_OUT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield BumpAutoState.CROSS_BUMP_OUT;
        }
        yield currentState;
      }
      case CROSS_BUMP_OUT -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield BumpAutoState.DRIVE_TO_OPPOSITE_HUB;
        }
        yield currentState;
      }
      case DRIVE_TO_OPPOSITE_HUB -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield BumpAutoState.WAIT_BEHIND_HUB;
        }
        yield currentState;
      }
      case WAIT_BEHIND_HUB -> {
        if (timeout(TRENCH_AUTO_CLEAR_WAIT_SECONDS)) {
          yield BumpAutoState.INTAKE_FIRST_CYCLE;
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
          yield BumpAutoState.CROSS_BUMP_TO_SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield BumpAutoState.DRIVE_TO_TOWER;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_TOWER -> {
        var speeds = robotManager.swerve.getFieldRelativeSpeeds();
        yield trailblazer.atGoal(robotManager.localization.getPose())
                && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1
                && Math.abs(speeds.omegaRadiansPerSecond) < 0.1
            ? BumpAutoState.SHOOT_1
            : currentState;
      }
      // Keep requesting the final shot until autonomous is disabled or ends.
      case SHOOT_1 -> currentState;
      case DONE -> BumpAutoState.DONE;
    };
  }

  @Override
  protected void whileInState(BumpAutoState newState) {
    if (getState() != BumpAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallIndex = trailblazer.getCurrentPointIndex();
    }

    switch (newState) {
      case PREPARE_BUMP_OUT_1 -> trailblazer.setActiveSegment(alignForBumpOut);
      case CROSS_BUMP_OUT -> {
        trailblazer.setActiveSegment(crossBumpOut);
        robotManager.intakeRequest();
        robotManager.powerManager.firstAutoSegmentRequest();
      }
      case DRIVE_TO_OPPOSITE_HUB, WAIT_BEHIND_HUB -> {
        trailblazer.setActiveSegment(driveToOppositeHub);
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
        robotManager.shooter.prepareScoreRequest(SHOOTER_WARMUP_DISTANCE_FROM_HUB);
      }
      case DRIVE_TO_TOWER -> {
        trailblazer.setActiveSegment(driveToTower);
        robotManager.warmupScoreRequest();
      }
      case SHOOT_1 -> {
        robotManager.prepareScoreRequest();
      }
      case DONE -> {}
    }
  }
}
