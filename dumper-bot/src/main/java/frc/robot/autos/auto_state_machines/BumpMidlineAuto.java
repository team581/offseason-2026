package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.BumpMidlineAutoState;
import frc.robot.robot_manager.RobotManager;

public class BumpMidlineAuto extends BaseImperativeAuto<BumpMidlineAutoState> {

  private final AutoSegment segment1DriveToMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.996, 6.0, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(11.912, 6.0, Rotation2d.kZero)),
              AutoPoint.ofRed(new Pose2d(10.218, 6.0, Rotation2d.kZero)))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment2IntakeAcrossMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.218, 5.8, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.218, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.218, 2.885, Rotation2d.kCW_90deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment3DriveBack =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.218, 2.885, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.218, 4.341, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(10.218, 5.482, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(11.912, 5.482, Rotation2d.k180deg)))
          .withLinearConstraints(3, 3)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment4DriveToShoot =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(13.751, 5.482, Rotation2d.kCW_90deg)),
              AutoPoint.ofRed(new Pose2d(14.305, 4.797, Rotation2d.fromDegrees(-145))))
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  public BumpMidlineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(BumpMidlineAutoState.SEGMENT_1_DRIVE_TO_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.996, 6.0, Rotation2d.kZero));
  }

  @Override
  protected BumpMidlineAutoState getNextState(BumpMidlineAutoState currentState) {

    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_1_DRIVE_TO_MIDLINE -> BumpMidlineAutoState.INTAKE_ACROSS_MIDLINE;
        case INTAKE_ACROSS_MIDLINE -> BumpMidlineAutoState.SEGMENT_3_DRIVE_BACK;
        case SEGMENT_3_DRIVE_BACK -> BumpMidlineAutoState.SEGMENT_4_DRIVE_TO_SHOOT;
        case SEGMENT_4_DRIVE_TO_SHOOT -> BumpMidlineAutoState.DONE;
        case DONE -> BumpMidlineAutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(BumpMidlineAutoState newState) {
    switch (newState) {
      case SEGMENT_1_DRIVE_TO_MIDLINE -> trailblazer.setActiveSegment(segment1DriveToMidline);
      case INTAKE_ACROSS_MIDLINE -> {
        //  robotManager.intakeRequest();
        trailblazer.setActiveSegment(segment2IntakeAcrossMidline);
      }
      case SEGMENT_3_DRIVE_BACK -> {
        trailblazer.setActiveSegment(segment3DriveBack);
        //  robotManager.cancelIntakeRequest();
      }
      case SEGMENT_4_DRIVE_TO_SHOOT -> {
        trailblazer.setActiveSegment(segment4DriveToShoot);
        //  robotManager.forceShootRequest();
      }
      case DONE -> {}
    }
  }
}
