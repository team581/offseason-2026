package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;

/** Safe default autonomous routine that commands no motion or mechanism actions. */
final class DoNothingAuto extends TrailblazerAuto<DoNothingAuto.State> {
  enum State {
    IDLE
  }

  DoNothingAuto(AutoContext context) {
    super(State.IDLE, Pose2d.kZero, context);
  }
}
