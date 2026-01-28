package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.IntegrationTestState;
import frc.robot.robot_manager.RobotManager;

public class IntegrationTest extends BaseImperativeAuto<IntegrationTestState> {

  private IntegrationTestState beforePauseState = IntegrationTestState.PAUSE;

  public IntegrationTest(RobotManager robotManager, Trailblazer trailblazer) {
    super(IntegrationTestState.DRIVE_TO_START, robotManager, trailblazer);
  }

  public void pauseRequest() {
    if (getState() == IntegrationTestState.PAUSE) {
      setStateFromRequest(beforePauseState);
    }

    beforePauseState = getState();
    setStateFromRequest(IntegrationTestState.PAUSE);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(Pose2d.kZero);
  }

  @Override
  protected IntegrationTestState getNextState(IntegrationTestState currentState) {
    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case DRIVE_TO_START -> IntegrationTestState.CLOSE_CENTERED_WITH_HUB;
        case CLOSE_CENTERED_WITH_HUB -> IntegrationTestState.PAUSE;
        case BACK_CENTERED_WITH_HUB -> IntegrationTestState.PAUSE;
        case RIGHT_TRENCH -> IntegrationTestState.PAUSE;
        case SHOOT_ON_MOVE_RIGHT_TRENCH_TO_MIDDLE_CENTERED_WITH_HUB -> IntegrationTestState.PAUSE;
        case PAUSE -> currentState;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(IntegrationTestState newState) {}
}
