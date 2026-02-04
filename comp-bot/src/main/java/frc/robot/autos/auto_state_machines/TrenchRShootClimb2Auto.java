package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.TrenchRShootClimb2AutoState;
import frc.robot.robot_manager.RobotManager;

@SuppressWarnings("unused")
public class TrenchRShootClimb2Auto extends BaseImperativeAuto<TrenchRShootClimb2AutoState> {

  private final AutoSegment driveToMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.2, 7.4, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(11.912, 7.4, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(10.1, 7.4, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(9.32, 7.4, Rotation2d.kZero)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment intakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.32, 6.0, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 5.8, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 2.885, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment driveBack =
  //update for trench
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.32, 2.885, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(9.32, 5.6, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(11.912, 5.6, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(12.9, 5.6, Rotation2d.k180deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

private final AutoSegment driveToShoot =
//update for this auto
    Trailblazer.segment(
        AutoPoint.ofRed(new Pose2d(0,0, Rotation2d.kZero))).withLinearConstraints(0, 0).untilFinished();

  private final AutoSegment driveToClimb =
  //update
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(13.751, 5.6, Rotation2d.fromDegrees(-134.0))),
              AutoPoint.ofRed(new Pose2d(14.305, 4.797, Rotation2d.fromDegrees(-164.0))))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  public TrenchRShootClimb2Auto(RobotManager robotManager, Trailblazer trailblazer) {
    super(TrenchRShootClimb2AutoState.DRIVE_TO_MIDLINE_1, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.2, 7.4, Rotation2d.kZero));
  }

  @Override
  protected TrenchRShootClimb2AutoState getNextState(TrenchRShootClimb2AutoState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case DRIVE_TO_MIDLINE_1 -> TrenchRShootClimb2AutoState.INTAKE_ACROSS_MIDLINE_1;
        case INTAKE_ACROSS_MIDLINE_1 -> TrenchRShootClimb2AutoState.DRIVE_BACK_1;
        case DRIVE_BACK_1 -> TrenchRShootClimb2AutoState.SHOOT_1;
        case SHOOT_1 -> TrenchRShootClimb2AutoState.DRIVE_TO_MIDLINE_2;
        case DRIVE_TO_MIDLINE_2 -> TrenchRShootClimb2AutoState.INTAKE_ACROSS_MIDLINE_2;
        case INTAKE_ACROSS_MIDLINE_2 -> TrenchRShootClimb2AutoState.DRIVE_BACK_2;
        case DRIVE_BACK_2 -> TrenchRShootClimb2AutoState.SHOOT_2;
        case SHOOT_2 -> TrenchRShootClimb2AutoState.DRIVE_TO_CLIMB;
        case DRIVE_TO_CLIMB -> TrenchRShootClimb2AutoState.CLIMB;
        case CLIMB -> TrenchRShootClimb2AutoState.DONE;
        case DONE -> TrenchRShootClimb2AutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(TrenchRShootClimb2AutoState newState) {
    switch (newState) {
      case DRIVE_TO_MIDLINE_1 -> trailblazer.setActiveSegment(driveToMidline);
      case INTAKE_ACROSS_MIDLINE_1 -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_1 -> {
        trailblazer.setActiveSegment(driveBack);
        robotManager.idleRequest();
      }
      case SHOOT_1 -> robotManager.prepareScoreRequest();
      case DRIVE_TO_MIDLINE_2 -> trailblazer.setActiveSegment(driveToMidline);
      case INTAKE_ACROSS_MIDLINE_2 -> {
        trailblazer.setActiveSegment(intakeAcrossMidline);
        robotManager.intakeRequest();
      }
      case DRIVE_BACK_2 -> {
        trailblazer.setActiveSegment(driveBack);
        robotManager.idleRequest();
      }
      case SHOOT_2 -> robotManager.prepareScoreRequest();
      case DRIVE_TO_CLIMB -> {
        trailblazer.setActiveSegment(driveToClimb);
        // raise climber as you shoot & drive to climb?
      }
      case CLIMB -> {
        // climb request
      }
      case DONE -> {}
    }
  }
}
