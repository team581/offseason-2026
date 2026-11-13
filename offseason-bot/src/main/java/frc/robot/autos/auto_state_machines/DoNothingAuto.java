package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autos.BaseImperativeAuto;
import frc.robot.autos.auto_state_machines.auto_state.DoNothingAutoState;
import frc.robot.robot_manager.RobotManager;

public class DoNothingAuto extends BaseImperativeAuto<DoNothingAutoState> {
  public DoNothingAuto(RobotManager robot, Trailblazer trailblazer) {
    super(DoNothingAutoState.DO_NOTHING, robot, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(
            BumpAutoStart.RED_STARTING_LINE_X + BumpAutoStart.HALF_BUMPER_METERS,
            FieldUtil.RED_HUB_CENTER.getY(),
            Rotation2d.kZero));
  }

  @Override
  protected void afterTransition(DoNothingAutoState newState) {
    switch (newState) {
      case DO_NOTHING -> {
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
    }
  }
}
