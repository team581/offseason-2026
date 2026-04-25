package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.autos.StuckOnBallRecovery;
import com.team581.math.PoseErrorTolerance;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.SpecialAutoState;
import frc.robot.config.FeatureFlags;
import frc.robot.robot_manager.RobotManager;

public class RightSpecialAuto extends BaseImperativeAuto<SpecialAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    CANCEL_INTAKE_RQ,
    READY_TO_SHOOT_FOR_2,
  }

  private BumpCrossingTracker bumpCrossingTracker;

  private static final double COLLISION_X_OFFSET = 0.5;
  private static final double MAX_CLUSTER_MAP_OFFSET = 0.35;

  private static final double MIDLINE_OFFSET = 0.0;

  private static final double BUMP_OFFSET = Units.inchesToMeters(5);

  private static final double SHOOTING_TIMEOUT_1 = 2.0;
  private static final double SHOOTING_TIMEOUT_2 = 5.0;
  private static final double STARTING_DELAY = 3.0;

  private final Timer autoTimer = new Timer();

  private boolean collisionEverDetected = false;

  private final AutoSegment intakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          9.956, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(7.572, 7.436, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(5.773 + 0.35, 6.229, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.4, 100)),
              AutoPoint.ofRed(new Pose2d(5.773 + 0.35, 4.298, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(7.572, 3.540, Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.177, FieldUtil.RED_HUB_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100)),
              AutoPoint.ofRed(new Pose2d(10.278, FieldUtil.RED_HUB_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)),
              AutoPoint.of(
                      () -> {
                        bumpCrossingTracker.bumpCrossRequest(
                            Point.ofRed(
                                new Pose2d(
                                    13.763,
                                    FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                                    Rotation2d.fromDegrees(45))),
                            FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg);
                        return Point.ofRed(
                            new Pose2d(
                                10.278,
                                FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(),
                                Rotation2d.fromDegrees(45)));
                      })
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(4.0))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          14.0,
                          FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() + BUMP_OFFSET,
                          Rotation2d.fromDegrees(40.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withLinearConstraints(4.5, 8)
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(2.0), Units.rotationsToRadians(2.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private final AutoSegment defaultSecondSegment =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          12.5, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.5, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 8.0)
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(
                  new Pose2d(10.5, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(9.5 - 1.0, 6.814, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.5 - 1.0, 4.4 + MIDLINE_OFFSET, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.6, 100))
                  .withAngularConstraints(
                      Units.rotationsToRadians(2.5), Units.rotationsToRadians(2.0))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8.0)
          .withAngularConstraints(Units.rotationsToRadians(4.0), Units.rotationsToRadians(3.0))
          .untilFinished(new PoseErrorTolerance(0.3, 100));

  private AutoSegment stuckOnBall =
      StuckOnBallRecovery.getRecoverySegment(
          () -> robotManager.localization.getPose(),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getPitch()),
          () -> Rotation2d.fromDegrees(robotManager.localization.imu.getRoll()));

  private SpecialAutoState storedStuckOnBallState = SpecialAutoState.STARTING_WITH_DELAY;
  private AutoSegment storedStuckOnBallAutoSegment = intakeAcrossMidline;
  private int storedStuckOnBallIndex = 0;

  // FOR SIM ONLY!!!
  private boolean firstStuckOnBall = false;

  public RightSpecialAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(SpecialAutoState.STARTING_WITH_DELAY, robotManager, trailblazer);

    this.bumpCrossingTracker = robotManager.localization.imu.bumpCrossingTracker;
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(13.0, 7.60, Rotation2d.kCW_90deg));
  }

  @Override
  protected void collectInputs() {
    super.collectInputs();
    if (!collisionEverDetected
        && getState() == SpecialAutoState.STARTING_WITH_DELAY
        && DriverStation.isEnabled()
        && robotManager.localization.imu.collisionDetected()) {
      collisionEverDetected = true;
      DogLog.log("RightSpecialAuto/CollisionDetected", collisionEverDetected);
    }
  }

  @Override
  protected SpecialAutoState getNextState(SpecialAutoState currentState) {
    if (FeatureFlags.UNBEACH_AUTO_IRL.getAsBoolean()
        || FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()) {
      switch (currentState) {
        case STARTING_WITH_DELAY, INTAKE_ACROSS_MIDLINE, DEFAULT_SECOND_INTAKE_SEGMENT -> {
          if (StuckOnBallRecovery.stuckOnBall(
              robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
            return SpecialAutoState.STUCK_ON_BALL_RECOVERY;
          }
        }
        default -> {}
      }
    }

    return switch (currentState) {
      case STUCK_ON_BALL_RECOVERY -> {
        if (!StuckOnBallRecovery.stuckOnBall(
            robotManager.localization.imu.getPitch(), robotManager.localization.imu.getRoll())) {
          yield storedStuckOnBallState;
        } else {
          yield currentState;
        }
      }
      case STARTING_WITH_DELAY -> {
        if (autoTimer.hasElapsed(STARTING_DELAY)) {
          yield SpecialAutoState.INTAKE_ACROSS_MIDLINE;
        } else {
          yield currentState;
        }
      }
      case INTAKE_ACROSS_MIDLINE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield SpecialAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && trailblazer.passedMarker(Markers.START_SHOOT_RQ)
            && bumpCrossingTracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
          yield SpecialAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if ((timeout(SHOOTING_TIMEOUT_1) && !robotManager.hopperManager.isShooting())
            || timeout(SHOOTING_TIMEOUT_2)) {
          yield SpecialAutoState.DEFAULT_SECOND_INTAKE_SEGMENT;
        } else {
          yield currentState;
        }
      }
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())
            && trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield SpecialAutoState.DONE;
        }
        yield currentState;
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield SpecialAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(SpecialAutoState newState) {
    if (getState() != SpecialAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallIndex = trailblazer.getCurrentPointIndex();
    }

    switch (newState) {
      case STUCK_ON_BALL_RECOVERY -> {
        trailblazer.setActiveSegment(stuckOnBall);

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.0)
            && RobotBase.isSimulation()) {
          robotManager.localization.imu.setPitch(0.0);
          robotManager.localization.imu.setRoll(0.0);
        }
      }
      case STARTING_WITH_DELAY -> {}
      case INTAKE_ACROSS_MIDLINE -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeAutoRequest();
        robotManager.powerManager.firstAutoSegmentRequest();

        if (FeatureFlags.UNBEACH_AUTO_SIM_ONLY.getAsBoolean()
            && timeout(1.5)
            && RobotBase.isSimulation()
            && !firstStuckOnBall) {
          firstStuckOnBall = true;
          robotManager.localization.imu.setPitch(-30.0);
        }
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
        if (RobotBase.isSimulation()) {
          if (timeout(0.5)) {
            robotManager.localization.imu.setPitch(-7.5);
            robotManager.localization.imu.setRoll(-7.5);
          }
          if (timeout(0.7)) {
            robotManager.localization.imu.setPitch(7.5);
            robotManager.localization.imu.setRoll(7.5);
          }
          if (timeout(1.0)) {
            robotManager.localization.imu.setPitch(0.0);
            robotManager.localization.imu.setRoll(0.0);
          }
        }
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        trailblazer.setActiveSegment(defaultSecondSegment);
        robotManager.intakeAutoRequest();
      }
      case DONE -> {}
    }

    if (DriverStation.isEnabled()) {
      autoTimer.start();
    }
  }

  @Override
  protected void beforeTransition(SpecialAutoState oldState, SpecialAutoState newState) {
    if (newState == SpecialAutoState.STUCK_ON_BALL_RECOVERY) {
      storedStuckOnBallState = oldState;
      DogLog.log("Trailblazer/StoredStuckOnBall/State", storedStuckOnBallState);
      DogLog.log("Trailblazer/StoredStuckOnBall/Index", storedStuckOnBallIndex);
    }

    if (oldState == SpecialAutoState.STUCK_ON_BALL_RECOVERY) {
      trailblazer.setActiveSegment(storedStuckOnBallAutoSegment, storedStuckOnBallIndex);
    }
  }

  @Override
  protected void afterTransition(SpecialAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        storedStuckOnBallAutoSegment = intakeAcrossMidline;
      }
      case STARTING_WITH_DELAY -> {
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1, SHOOT_1 -> {
        storedStuckOnBallAutoSegment = driveBackAndShootOne;
      }
      case DEFAULT_SECOND_INTAKE_SEGMENT -> {
        storedStuckOnBallAutoSegment = defaultSecondSegment;
        robotManager.idleRequest();
      }
      case DONE -> {
        robotManager.idleRequest();
      }
      case STUCK_ON_BALL_RECOVERY -> {}
    }
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

  // Only use for lane 0 and 1 since we don't
  private Point getClusterShiftedPoint(Point point) {
    var targetCluster = robotManager.clusterMap.getBestClusterPose();

    if (targetCluster.isEmpty()) {
      return point;
    }

    Pose2d clusterPose = targetCluster.orElseThrow();
    Pose2d basePose = point.getPose();

    double clampedX =
        MathUtil.clamp(
            clusterPose.getX(),
            basePose.getX() - MAX_CLUSTER_MAP_OFFSET,
            basePose.getX() + MAX_CLUSTER_MAP_OFFSET);

    return FmsUtil.isRedAlliance()
        ? Point.ofRed(new Pose2d(clampedX, basePose.getY(), basePose.getRotation()))
        : Point.ofBlue(new Pose2d(clampedX, basePose.getY(), basePose.getRotation()));
  }
}
