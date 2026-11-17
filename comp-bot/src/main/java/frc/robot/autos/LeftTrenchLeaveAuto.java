package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;

/** Drives straight out of the left (depot-side) trench toward the alliance wall. */
public final class LeftTrenchLeaveAuto extends TrailblazerAuto<LeftTrenchLeaveAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-left-trench")
          .stopAt(CompAutoField.offset(CompAutoField.LEFT_TRENCH_START, -TRAVEL_METERS, 0.0))
          .build();

  LeftTrenchLeaveAuto(AutoContext context) {
    super(State.LEAVE_TRENCH, CompAutoField.LEFT_TRENCH_START, context, LEAVE_TRENCH_SEGMENT);
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
