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
import frc.robot.autos.auto_state_machines.auto_state.BumpSweepAutoState;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.BumpCrossingTracker;
import frc.robot.robot_manager.RobotManager;

public class RightBumpSweepAuto extends BaseImperativeAuto<BumpSweepAutoState> {

  public enum Markers {
    PRIORITIZE_INTAKE,
    START_SHOOT_RQ,
    READY_TO_CROSS_BUMP
  }

  private static final double COLLISION_X_OFFSET = -0.1;

  private static final double BUMP_OFFSET = -0.13;

  static final double SHOOT_X = 13.9;
  static final double RETURN_BUMP_LANDING_X = 13.25;
  // Keep the sweep on its own side, including the rotated bumper envelope.
  static final double SWEEP_MIN_RED_Y = FieldUtil.FIELD_WIDTH_Y - 3.3;

  // Centered between the hub and climb tower, just forward of the tower preset.
  static final Point TOWER_SHOOT_GOAL =
      Point.ofRed(new Pose2d(14.8, FieldUtil.FIELD_WIDTH_Y / 2.0, Rotation2d.kZero));

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

  private BumpCrossingTracker bumpCrossingTracker;

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeFirstCycle =
      Trailblazer.segment(
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(
                                  9.200,
                                  FieldUtil.FIELD_WIDTH_Y - 1.850,
                                  Rotation2d.fromDegrees(155)))))
                  .withMarker(Markers.PRIORITIZE_INTAKE)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(
                                  8.316,
                                  FieldUtil.FIELD_WIDTH_Y - 1.850,
                                  Rotation2d.fromDegrees(-105)))))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(
                                  8.316, FieldUtil.FIELD_WIDTH_Y - 2.500, Rotation2d.kCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(new Pose2d(8.316, SWEEP_MIN_RED_Y, Rotation2d.kCW_90deg))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.35, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(8.744, SWEEP_MIN_RED_Y, Rotation2d.fromDegrees(-35)))))
                  .withLinearConstraints(2.5, 3.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 45)),
              AutoPoint.of(() ->
                      getCollisionPoint(
                          Point.ofRed(
                              new Pose2d(
                                  9.556,
                                  FieldUtil.FIELD_WIDTH_Y - 3.050,
                                  Rotation2d.fromDegrees(25)))))
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

  private BumpSweepAutoState storedStuckOnBallState = BumpSweepAutoState.INTAKE_FIRST_CYCLE;

  private AutoSegment storedStuckOnBallAutoSegment = intakeFirstCycle;
  private int storedStuckOnBallIndex = 0;
  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;

  public RightBumpSweepAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(BumpSweepAutoState.PREPARE_BUMP_OUT_1, robotManager, trailblazer);

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
  protected void afterTransition(BumpSweepAutoState newState) {
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
      case INTAKE_FIRST_CYCLE -> {
        storedStuckOnBallAutoSegment = intakeFirstCycle;
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        storedStuckOnBallAutoSegment = crossBumpToShootOne;
        bumpCrossingTracker.bumpCrossRequest(
            Point.ofRed(
                new Pose2d(
                    RETURN_BUMP_LANDING_X,
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
        robotManager.cancelIntakeRequest();
        robotManager.idleRequest();
        trailblazer.clearActiveSegment();
      }
      case STUCK_ON_BALL_RECOVERY -> {}
    }
  }

  @Override
  protected void beforeTransition(BumpSweepAutoState oldState, BumpSweepAutoState newState) {

    if (newState == BumpSweepAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
      DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
      DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);
    }

    if (oldState == BumpSweepAutoState.STUCK_ON_BALL_RECOVERY) {
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
        && getState() == BumpSweepAutoState.INTAKE_FIRST_CYCLE
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
      DogLog.log("RightBumpSweepAuto/CollisionDetected", collisionEverDetected);
    }
  }

  @Override
  protected BumpSweepAutoState getNextState(BumpSweepAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
        || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()) {
      switch (currentState) {
        case INTAKE_FIRST_CYCLE -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return BumpSweepAutoState.STUCK_ON_BALL_RECOVERY;
          }
        }
        default -> {}
      }
    }

    return switch (currentState) {
      case PREPARE_BUMP_OUT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield BumpSweepAutoState.CROSS_BUMP_OUT;
        }
        yield currentState;
      }
      case CROSS_BUMP_OUT -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield BumpSweepAutoState.INTAKE_FIRST_CYCLE;
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
          yield BumpSweepAutoState.CROSS_BUMP_TO_SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        // Drive clear of the bump as soon as the landing is detected.
        if (bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield BumpSweepAutoState.DRIVE_TO_TOWER;
        } else {
          yield currentState;
        }
      }
      case DRIVE_TO_TOWER -> {
        var speeds = robotManager.swerve.getFieldRelativeSpeeds();
        yield trailblazer.atGoal(robotManager.localization.getPose())
                && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1
                && Math.abs(speeds.omegaRadiansPerSecond) < 0.1
            ? BumpSweepAutoState.SHOOT_1
            : currentState;
      }
      // Keep shooting at standstill until autonomous ends.
      case SHOOT_1 -> currentState;
      case DONE -> BumpSweepAutoState.DONE;
    };
  }

  @Override
  protected void whileInState(BumpSweepAutoState newState) {
    if (getState() != BumpSweepAutoState.STUCK_ON_BALL_RECOVERY) {
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
      case DONE -> {}
    }
  }
}
