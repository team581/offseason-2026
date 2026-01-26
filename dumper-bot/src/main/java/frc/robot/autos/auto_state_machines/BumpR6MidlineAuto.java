package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_states.BumpR6MidlineAutoState;
import frc.robot.robot_manager.RobotManager;

public class BumpR6MidlineAuto extends BaseImperativeAuto<BumpR6MidlineAutoState> {

  public BumpR6MidlineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(BumpR6MidlineAutoState.SEGMENT_1_DRIVE_TO_MIDLINE, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(Pose2d.kZero);
  }

  @Override
  protected BumpR6MidlineAutoState getNextState(BumpR6MidlineAutoState currentState) {

    if (trailblazer.atGoal(robotManager.localization.getPose())) {
      return switch (currentState) {
        case SEGMENT_1_DRIVE_TO_MIDLINE -> BumpR6MidlineAutoState.INTAKE;
        case INTAKE -> BumpR6MidlineAutoState.SEGMENT_2_DRIVE_ACROSS_MIDLINE;
        case SEGMENT_2_DRIVE_ACROSS_MIDLINE -> BumpR6MidlineAutoState.SEGMENT_3_DRIVE_BACK;
        case SEGMENT_3_DRIVE_BACK -> BumpR6MidlineAutoState.SEGMENT_4_DRIVE_TO_SHOOT;
        case SEGMENT_4_DRIVE_TO_SHOOT -> BumpR6MidlineAutoState.DONE;
        case DONE -> BumpR6MidlineAutoState.DONE;
      };
    }
    return currentState;
  }

  @Override
  protected void whileInState(BumpR6MidlineAutoState newState) {}
}
