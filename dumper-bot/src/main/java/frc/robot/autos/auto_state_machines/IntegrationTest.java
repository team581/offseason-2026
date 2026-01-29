package frc.robot.autos.auto_state_machines;

import com.team581.GlobalConfig;
import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.IntegrationTestState;
import frc.robot.robot_manager.RobotManager;

public class IntegrationTest extends BaseImperativeAuto<IntegrationTestState> {

  private IntegrationTestState beforePauseState = IntegrationTestState.PAUSED;
  private static final double MAX_VELOCITY = 1.0;
  private static final double MAX_ACCELERATION = 1.0;

  private static final Pose2d RED_START_POSE =
      FieldUtil.HUB_POSE
          .redPose()
          .plus(new Transform2d(Units.inchesToMeters(60.0), 0.0, Rotation2d.kZero));

  private final AutoSegment segment1DriveToStart =
      Trailblazer.segment(AutoPoint.ofRed(RED_START_POSE))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment2CloseCenteredWithHub =
      Trailblazer.segment(
              AutoPoint.ofRed(RED_START_POSE.plus(new Transform2d(-0.25, 0.0, Rotation2d.kZero))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment3BackCenteredWithHub =
      Trailblazer.segment(
              // Climb area
              AutoPoint.ofRed(new Pose2d(14.929, RED_START_POSE.getY(), Rotation2d.kZero)))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment4RightTrench =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(14.558, 6.399, Rotation2d.fromDegrees(110))),
              AutoPoint.ofRed(new Pose2d(13.36, 7.29, Rotation2d.fromDegrees(-90))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  private final AutoSegment segment5RightTrench =
      Trailblazer.segment(
              // Climb area
              AutoPoint.ofRed(new Pose2d(14.929, RED_START_POSE.getY(), Rotation2d.k180deg)),
              AutoPoint.ofRed(new Pose2d(13.36, 2.54, Rotation2d.fromDegrees(-45))))
          .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
          .untilFinished(new PoseErrorTolerance(0.05, 3));

  public IntegrationTest(RobotManager robotManager, Trailblazer trailblazer) {
    super(IntegrationTestState.SEGMENT_1_DRIVE_TO_START, robotManager, trailblazer);
  }

  public void pauseRequest() {
    if (getState() == IntegrationTestState.PAUSED) {
      setStateFromRequest(beforePauseState);
    }

    beforePauseState = getState();
    setStateFromRequest(IntegrationTestState.PAUSED);
  }

  public void skipRequest() {
    setStateFromRequest(getState().nextState());
  }

  public void previousRequest() {
    setStateFromRequest(getState().previousState());
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(Pose2d.kZero);
  }

  @Override
  protected IntegrationTestState getNextState(IntegrationTestState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_2_CLOSE_CENTERED_WITH_HUB ->
            timeout(2.0) ? IntegrationTestState.SEGMENT_3_BACK_CENTERED_WITH_HUB : currentState;
        case PAUSED -> currentState;
        default -> currentState.nextState();
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(IntegrationTestState newState) {
    switch (newState) {
      case SEGMENT_1_DRIVE_TO_START -> {
        robotManager.swerve.trailblazerDriveRequest(segment1DriveToStart);
        robotManager.idleRequest();
      }
      case SEGMENT_2_CLOSE_CENTERED_WITH_HUB -> {
        robotManager.swerve.trailblazerDriveRequest(segment2CloseCenteredWithHub);
        robotManager.scoreRequest();
      }
      case SEGMENT_3_BACK_CENTERED_WITH_HUB -> {
        robotManager.swerve.trailblazerDriveRequest(segment3BackCenteredWithHub);
        robotManager.scoreRequest();
      }
      case SEGMENT_4_RIGHT_TRENCH -> {
        robotManager.swerve.trailblazerDriveRequest(segment4RightTrench);
        robotManager.scoreRequest();
      }
      case SEGMENT_5_LEFT_RAMP -> {
        robotManager.swerve.trailblazerDriveRequest(segment5RightTrench);
        robotManager.scoreRequest();
      }
      case PAUSED -> {
        robotManager.swerve.normalDriveRequest();
        robotManager.idleRequest();
      }
    }
  }
}
