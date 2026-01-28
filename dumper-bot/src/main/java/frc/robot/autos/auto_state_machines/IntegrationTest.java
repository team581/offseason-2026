package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
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
  protected void whileInState(IntegrationTestState newState) {}
}
