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
        case CLOSE_CENTERED_WITH_HUB ->
            timeout(2.0) ? IntegrationTestState.BACK_CENTERED_WITH_HUB : currentState;
        case PAUSE -> currentState;
        default -> currentState.nextState();
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(IntegrationTestState newState) {}
}
