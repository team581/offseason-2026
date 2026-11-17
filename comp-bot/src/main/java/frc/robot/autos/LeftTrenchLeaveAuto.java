package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.CompAutoField;

/** Drives straight out of the left (depot-side) trench toward the alliance wall. */
public final class LeftTrenchLeaveAuto extends TrailblazerAuto<LeftTrenchLeaveAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;

  static final AutoSegment LEAVE_TRENCH_SEGMENT = leaveTrench(CompAutoField.LEFT_TRENCH_START);

  public static AutoSegment homeLeaveSegment() {
    return leaveTrench(CompAutoField.HOME_LEFT_TRENCH_START);
  }

  private static AutoSegment leaveTrench(Pose2d start) {
    return Trailblazer.segment("leave-left-trench")
        .stopAt(CompAutoField.offset(start, -TRAVEL_METERS, 0.0))
        .build();
  }

  private final AutoSegment segment;

  private LeftTrenchLeaveAuto(AutoContext context, AutoSegment segment) {
    super(
        State.LEAVE_TRENCH,
        context.clamped() ? CompAutoField.HOME_LEFT_TRENCH_START : CompAutoField.LEFT_TRENCH_START,
        context,
        segment);
    this.segment = segment;
  }

  LeftTrenchLeaveAuto(AutoContext context) {
    this(context, context.clamped() ? homeLeaveSegment() : LEAVE_TRENCH_SEGMENT);
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case LEAVE_TRENCH -> {
        drive(segment);
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
