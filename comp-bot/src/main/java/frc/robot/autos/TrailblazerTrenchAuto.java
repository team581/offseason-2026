package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;

final class TrailblazerTrenchAuto extends TrailblazerAuto<TrailblazerTrenchAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;
  private static final Pose2d START = CompAutoField.RIGHT_TRENCH_START;

  private static final AutoSegment LEAVE_TRENCH =
      Trailblazer.segment("leave-trench")
          .withLimits(MotionLimits.robotMaximum())
          .stopAt(new Pose2d(START.getX() - TRAVEL_METERS, START.getY(), START.getRotation()))
          .build();

  static Pose2d start(Alliance alliance) {
    return CompAutoField.pose(START, alliance);
  }

  static void warmup(Alliance alliance) {
    // Exercise the first planner tick without changing the live planner or commanding the swerve.
    var planner = CompTrailblazer.create(() -> 0);
    planner.activate(
        CompAutoField.segment(LEAVE_TRENCH, alliance), CompAutoField.clearance(alliance));
    planner.update(RobotStateSnapshot.measured(0, start(alliance), new ChassisSpeeds(), true));
  }

  TrailblazerTrenchAuto(Trailblazer trailblazer, Alliance alliance) {
    super(State.LEAVE_TRENCH, trailblazer, alliance);
  }

  @Override
  protected void afterTransition(State state) {
    if (state == State.LEAVE_TRENCH) {
      activate(LEAVE_TRENCH);
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
