package frc.robot.autos;

/** Safe default autonomous routine that commands no motion or mechanism actions. */
final class DoNothingAuto extends TrailblazerAuto<DoNothingAuto.State> {
  enum State {
    IDLE
  }

  DoNothingAuto(AutoContext context) {
    super(State.IDLE, context);
  }
}
