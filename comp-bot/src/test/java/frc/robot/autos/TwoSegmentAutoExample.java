package frc.robot.autos;

import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.robot_manager.RobotManager;

/** An example for authors. The competition chooser never instantiates it. */
final class TwoSegmentAutoExample extends TrailblazerAuto<TwoSegmentAutoExample.State> {
  enum Marker {
    AT_APPROACH
  }

  enum State {
    APPROACH,
    HOME_DEPLOY,
    CONTINUE,
    DONE
  }

  private final RobotManager robot;
  private final com.team581.trailblazer3.model.AutoSegment approach =
      Trailblazer.segment("approach")
          .addGoal(Goal.stop(new Pose2d(2, 2, Rotation2d.kZero)).marker(Marker.AT_APPROACH))
          .build();
  private final com.team581.trailblazer3.model.AutoSegment continueDrive =
      Trailblazer.segment("continue").stopAt(new Pose2d(3, 2, Rotation2d.kZero)).build();

  TwoSegmentAutoExample(RobotManager robot, Trailblazer trailblazer, Alliance alliance) {
    super(State.APPROACH, trailblazer, alliance);
    this.robot = robot;
  }

  @Override
  protected void afterTransition(State state) {
    switch (state) {
      case APPROACH -> activate(approach);
      case HOME_DEPLOY -> robot.homeDeployInAutoRequest();
      case CONTINUE -> activate(continueDrive);
      case DONE -> {}
    }
  }

  @Override
  protected State getNextState(State state) {
    return switch (state) {
      case APPROACH -> trailblazer.passedMarker(Marker.AT_APPROACH) ? State.HOME_DEPLOY : state;
      case HOME_DEPLOY -> robot.hopperManager.deploy.getState().isHoming() ? state : State.CONTINUE;
      case CONTINUE -> trailblazer.atGoal() ? State.DONE : state;
      case DONE -> state;
    };
  }
}
