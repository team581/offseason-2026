package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.RightCircleSoMAutoState;
import frc.robot.robot_manager.RobotManager;

public class RightCircleSoMAuto extends BaseImperativeAuto<RightCircleSoMAutoState> {

  public enum Markers {
    START_SHOOT_RQ,
    CANCEL_INTAKE_RQ
  }

  private final AutoSegment intakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          10.489, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(8.640, 6.653, Rotation2d.fromDegrees(-134.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(7.983, 5.593, Rotation2d.fromDegrees(-86.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(8.280, 4.475, Rotation2d.fromDegrees(-50.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(9.31, 4.31, Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(9.975, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.174, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))        
                  .withTransitionTolerance(new PoseErrorTolerance(0.5, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1.0), Units.rotationsToRadians(2))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeBehindHub =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
              .withTransitionTolerance(new PoseErrorTolerance(0.1)),
              AutoPoint.ofRed(
                      new Pose2d(
                          9.974, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.fromDegrees(-152.0)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2)),
              AutoPoint.ofRed(new Pose2d(9.6, 5.7, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1)),
              AutoPoint.ofRed(new Pose2d(9.4, 4.49, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100)),
              AutoPoint.ofRed(new Pose2d(10.335, 4.49, Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.1, 100))
                  .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1.0)),
              AutoPoint.ofRed(new Pose2d(10.335, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1.0))
                  .withLinearConstraints(1.75, 4.0),
               AutoPoint.ofRed(
                      new Pose2d(10.77, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1))
                  .withLinearConstraints(1.75, 4),
              AutoPoint.ofRed(
                      new Pose2d(
                          11.878, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100))
                  .withMarker(Markers.CANCEL_INTAKE_RQ)
                  .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1))
                  .withLinearConstraints(1.75, 4))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1.0), Units.rotationsToRadians(2))
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveBackAndShootOne =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))    
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 10))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withMarker(Markers.START_SHOOT_RQ),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(1.0, 20)
          .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveBackAndShootTwo =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(13.0, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kZero))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY(), Rotation2d.kCCW_90deg))  
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withLinearConstraints(4.5, 8),
              AutoPoint.ofRed(
                      new Pose2d(
                          13.709, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 100))
                  .withMarker(Markers.START_SHOOT_RQ))
          .withLinearConstraints(1.0, 20)
          .withAngularConstraints(Units.rotationsToRadians(0.5), Units.rotationsToRadians(1))
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveBackToNeutralZone =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          13.0, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(10.174, 7.28, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)),
              AutoPoint.ofRed(new Pose2d(9.8, 5.708, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3, 100)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(1.0), Units.rotationsToRadians(2))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  public RightCircleSoMAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightCircleSoMAutoState.INTAKE_ACROSS_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg));
  }

  @Override
  protected RightCircleSoMAutoState getNextState(RightCircleSoMAutoState currentState) {
    return switch (currentState) {
      case INTAKE_ACROSS_MIDLINE -> {
        if (trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield RightCircleSoMAutoState.DRIVE_BACK_1;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_1 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightCircleSoMAutoState.SHOOT_1;
        } else {
          yield currentState;
        }
      }
      case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.5)) {
          yield RightCircleSoMAutoState.INTAKE_BEHIND_HUB;
        } else {
          yield currentState;
        }
      }
      case INTAKE_BEHIND_HUB -> {
        if (trailblazer.passedMarker(Markers.CANCEL_INTAKE_RQ)) {
          yield RightCircleSoMAutoState.DRIVE_BACK_2;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_2 -> {
        if (trailblazer.passedMarker(Markers.START_SHOOT_RQ)) {
          yield RightCircleSoMAutoState.SHOOT_2;
        } else {
          yield currentState;
        }
      }
      case SHOOT_2 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.5)) {
          yield RightCircleSoMAutoState.DRIVE_BACK_TO_NEUTRAL_ZONE;
        } else {
          yield currentState;
        }
      }
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightCircleSoMAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield RightCircleSoMAutoState.DONE;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightCircleSoMAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeAutoRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBackAndShootOne);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case INTAKE_BEHIND_HUB -> {
        trailblazer.setActiveSegment(intakeBehindHub);
        robotManager.intakeAutoRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBackAndShootTwo);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_2 -> robotManager.prepareScoreRequest();
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> trailblazer.setActiveSegment(driveBackToNeutralZone);
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(RightCircleSoMAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case INTAKE_BEHIND_HUB -> {
        robotManager.idleRequest();
      }
      case DRIVE_BACK_2 -> {}
      case SHOOT_2 -> {}
      case DRIVE_BACK_TO_NEUTRAL_ZONE -> {
        robotManager.idleRequest();
      }
      case DONE -> {
        robotManager.idleRequest();
      }
    }
  }
}
