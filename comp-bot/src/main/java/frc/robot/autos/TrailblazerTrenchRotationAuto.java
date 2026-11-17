package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.CompAutoField;

/**
 * Leaves the trench perimeter and rotates 180 degrees counterclockwise while strafing toward the
 * interior of the field.
 */
final class TrailblazerTrenchRotationAuto
    extends TrailblazerAuto<TrailblazerTrenchRotationAuto.State> {
  enum State {
    DRIVE_AND_TURN,
    DONE
  }

  private static final Pose2d START = CompAutoField.RIGHT_TRENCH_EXIT;
  // Positive blue-field Y is left as seen from the driver station, toward the field interior.
  private static final Pose2d END =
      new Pose2d(START.getX(), START.getY() + 3.0, START.getRotation().plus(Rotation2d.k180deg));

  static final AutoSegment SEGMENT =
      Trailblazer.segment("trench-rotation").stopAt(END, Goal::counterClockwise).build();

  TrailblazerTrenchRotationAuto(AutoContext context) {
    super(State.DRIVE_AND_TURN, START, context, SEGMENT);
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case DRIVE_AND_TURN -> drive(SEGMENT);
      case DONE -> stopDriving();
    }
  }

  @Override
  protected State getNextState(State currentState) {
    return currentState == State.DRIVE_AND_TURN && atGoal() ? State.DONE : currentState;
  }
}
