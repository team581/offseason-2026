package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.RightSwoopShootClimbAutoState;
import frc.robot.climber.ClimbLocation;
import frc.robot.robot_manager.ClimbAssist;
import frc.robot.robot_manager.RobotManager;

@SuppressWarnings("unused")

public class RightSwoopShootClimbAuto extends BaseImperativeAuto<RightSwoopShootClimbAutoState> {

  private final AutoSegment intakeAcrossMidlineOne =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.358, 7.4, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(8.859, 7.15, Rotation2d.fromDegrees(-140.0))),
              AutoPoint.ofRed(new Pose2d(7.778, 5.665, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.227, 4.549, Rotation2d.kZero)))
        .withLinearConstraints(4.5, 10)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment intakeAcrossMidlineTwo =
    // TODO: update all poses and tolerances
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.2, 7.5, Rotation2d.fromDegrees(-150.0))),
              AutoPoint.ofRed(new Pose2d(9.0, 6.865, Rotation2d.fromDegrees(-126.0))),
              AutoPoint.ofRed(new Pose2d(8.852, 5.2, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(2, 3)))
          .withLinearConstraints(4.5, 10)
          .untilFinished(new PoseErrorTolerance(0.1, 3));

  private final AutoSegment driveBackAndShoot =
      Trailblazer.segment(
        AutoPoint.ofRed(new Pose2d(9.505, 5.933, Rotation2d.kCW_90deg)),
        AutoPoint.ofRed(new Pose2d(10.293, 7.411, Rotation2d.k180deg)))
          .withLinearConstraints(4.5, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  private final AutoSegment driveToShootOne =
    // TODO: update all poses and tolerances
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.3, 7.55, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(12.8, 7.57, Rotation2d.k180deg))
                  .withLinearConstraints(4.5, 10))
          .untilFinished(new PoseErrorTolerance(0.3, 3));

  private final AutoSegment driveToShootTwo =
    // TODO: update all poses and tolerances
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(11.3, 7.55, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.5, 7.5, Rotation2d.k180deg)))
          .withLinearConstraints(4.5, 10)
          .untilFinished(new PoseErrorTolerance(0.5, 3));

  public RightSwoopShootClimbAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(RightSwoopShootClimbAutoState.INTAKE_ACROSS_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.15, 7.55, Rotation2d.k180deg));
  }

  @Override
  protected RightSwoopShootClimbAutoState getNextState(RightSwoopShootClimbAutoState currentState) {
      return switch (currentState) {
// super broken; still deciding where we're going to check for markers & where to check for reaching end pose in segment
        case INTAKE_ACROSS_MIDLINE_1 -> { 
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
            yield RightSwoopShootClimbAutoState.DRIVE_BACK_1;
        } else {
            yield currentState;
        }
    }
        case DRIVE_BACK_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
            yield RightSwoopShootClimbAutoState.SHOOT_1;
        } else {
            yield currentState;
        }
    }
        case SHOOT_1 -> {
        if (trailblazer.atGoal(robotManager.localization.getPose()) && timeout(3.0)) {
            yield RightSwoopShootClimbAutoState.INTAKE_ACROSS_MIDLINE_2;
        } else {
            yield currentState;
        }}
        case INTAKE_ACROSS_MIDLINE_2 -> RightSwoopShootClimbAutoState.DRIVE_BACK_2;
        case DRIVE_BACK_2 -> RightSwoopShootClimbAutoState.SHOOT_2;
        case SHOOT_2 -> RightSwoopShootClimbAutoState.DRIVE_TO_CLIMB;
        case DRIVE_TO_CLIMB -> RightSwoopShootClimbAutoState.CLIMB;
        case CLIMB -> RightSwoopShootClimbAutoState.DONE;
        case DONE -> RightSwoopShootClimbAutoState.DONE;
      

      default -> currentState;
    };
  }

  @Override
  protected void whileInState(RightSwoopShootClimbAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineOne);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
   //     trailblazer.setActiveSegment(driveBack);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_1 -> {
        trailblazer.setActiveSegment(driveToShootOne);
        robotManager.prepareScoreRequest();
      }
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidlineTwo);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
     //   trailblazer.setActiveSegment(driveBack);
        robotManager.cancelIntakeRequest();
      }
      case SHOOT_2 -> {
        trailblazer.setActiveSegment(driveToShootTwo);
        robotManager.prepareScoreRequest();
      }
      case DRIVE_TO_CLIMB -> {
        robotManager.startAutoClimbSequence();
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }

  @Override
  protected void afterTransition(RightSwoopShootClimbAutoState newState) {
    switch (newState) {
      case INTAKE_ACROSS_MIDLINE_1 -> {
        robotManager.homeDeployRequest();
        robotManager.homeShooterHoodRequest();
      }
      case DRIVE_BACK_1 -> {}
      case SHOOT_1 -> {}
      case INTAKE_ACROSS_MIDLINE_2 -> {}
      case DRIVE_BACK_2 -> {}
      case SHOOT_2 -> {}
      case DRIVE_TO_CLIMB -> {
        trailblazer.setActiveSegment(
            ClimbAssist.getClimbAssistSegment(
                robotManager.localization.getPose(), ClimbLocation.CLOSEST));
      }
      case CLIMB -> {}
      case DONE -> {}
    }
  }
}
