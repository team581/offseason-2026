package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;

final class TrailblazerTrenchAuto extends TrailblazerAuto<TrailblazerTrenchAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;
  private static final Pose2d START = CompAutoField.RIGHT_TRENCH_START;

  static Pose2d start(Alliance alliance) {
    return CompAutoField.pose(START, alliance);
  }

  private final AutoSegment leaveTrench =
      Trailblazer.segment("leave-trench")
          .withLimits(MotionLimits.robotMaximum())
          .stopAt(new Pose2d(START.getX() - TRAVEL_METERS, START.getY(), START.getRotation()))
          .build();

  TrailblazerTrenchAuto(Trailblazer trailblazer, Alliance alliance) {
    super(State.LEAVE_TRENCH, trailblazer, alliance);
  }

  @Override
  protected void afterTransition(State state) {
    if (state == State.LEAVE_TRENCH) {
      activate(leaveTrench);
    }
  }

  @Override
  protected State getNextState(State state) {
    return state == State.LEAVE_TRENCH && trailblazer.atGoal() ? State.DONE : state;
  }

  boolean acceptsStart(Pose2d pose) {
    var expected = start(alliance);
    return pose.getTranslation().getDistance(expected.getTranslation()) <= 0.08
        && Math.abs(pose.getRotation().minus(expected.getRotation()).getDegrees()) <= 5;
  }
}
