package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.CompAutoField;

/** Drives straight out of the right trench toward the alliance wall. */
final class TrailblazerTrenchAuto extends TrailblazerAuto<TrailblazerTrenchAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;

  static final Pose2d START = CompAutoField.RIGHT_TRENCH_START;
  static final Pose2d END = CompAutoField.offset(START, -TRAVEL_METERS, 0.0);

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-right-trench").stopAt(END).build();

  TrailblazerTrenchAuto(AutoContext context) {
    super(State.LEAVE_TRENCH, START, context, LEAVE_TRENCH_SEGMENT);
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case LEAVE_TRENCH -> {
        drive(LEAVE_TRENCH_SEGMENT);
        AutoActions.homeSubsystems(robot);
      }
      case DONE -> stopDriving();
    }
  }

  @Override
  protected State getNextState(State currentState) {
    return currentState == State.LEAVE_TRENCH && atGoal() ? State.DONE : currentState;
  }
}
