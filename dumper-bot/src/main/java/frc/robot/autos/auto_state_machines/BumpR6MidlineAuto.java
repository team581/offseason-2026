package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.BumpR6MidlineAutoState;
import frc.robot.robot_manager.RobotManager;

public class BumpR6MidlineAuto extends BaseImperativeAuto<BumpR6MidlineAutoState> {

  private final AutoSegment segment1DriveToMidline =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(12.996, 6.0, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(11.912, 6.0, Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(10.218, 6.0, Rotation2d.k180deg)))
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

  public BumpR6MidlineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(BumpR6MidlineAutoState.SEGMENT_1_DRIVE_TO_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(new Pose2d(12.996, 6.0, Rotation2d.k180deg));
  }

  @Override
  protected BumpR6MidlineAutoState getNextState(BumpR6MidlineAutoState currentState) {

    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_1_DRIVE_TO_MIDLINE -> BumpR6MidlineAutoState.INTAKE_ACROSS_MIDLINE;
        case INTAKE_ACROSS_MIDLINE -> BumpR6MidlineAutoState.SEGMENT_3_DRIVE_BACK;
        case SEGMENT_3_DRIVE_BACK -> BumpR6MidlineAutoState.SEGMENT_4_DRIVE_TO_SHOOT;
        case SEGMENT_4_DRIVE_TO_SHOOT -> BumpR6MidlineAutoState.DONE;
        case DONE -> BumpR6MidlineAutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(BumpR6MidlineAutoState newState) {
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
