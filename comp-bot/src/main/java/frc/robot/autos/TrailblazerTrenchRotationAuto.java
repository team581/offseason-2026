package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.HeadingPlan.TurnDirection;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;

final class TrailblazerTrenchRotationAuto
    extends TrailblazerAuto<TrailblazerTrenchRotationAuto.State> {
  enum State {
    CLEAR_WALL,
    DRIVE_AND_TURN,
    DONE
  }

  private static final Pose2d START = TrailblazerTrenchAuto.END;
  // Positive blue-field Y is left as seen from the driver station, toward the field interior.
  private static final Pose2d END =
      new Pose2d(START.getX(), START.getY() + 3.0, START.getRotation().plus(Rotation2d.k180deg));

  // Trailblazer checks the entire heading sweep at every point in the turning segment.
  private static final Pose2d TURN_START =
      new Pose2d(START.getX(), START.getY() + 0.20, START.getRotation());
  private static final AutoSegment CLEAR_WALL =
      Trailblazer.segment("trench-rotation-clear-wall").stopAt(TURN_START).build();

  private static final AutoSegment DRIVE_AND_TURN =
      Trailblazer.segment("trench-rotation-drive-and-turn")
          .addGoal(
              Goal.stop(END)
                  .heading(HeadingPlan.to(END.getRotation()).direction(TurnDirection.CCW)))
          .build();

  static Pose2d start(Alliance alliance) {
    return CompAutoField.pose(START, alliance);
  }

  static void warmup(Alliance alliance) {
    var planner = CompTrailblazer.create(() -> 0);
    var clearance = CompAutoField.clearance(alliance);
    planner.activate(CompAutoField.segment(CLEAR_WALL, alliance), clearance);
    planner.update(RobotStateSnapshot.measured(0, start(alliance), new ChassisSpeeds(), true));
    planner.activate(CompAutoField.segment(DRIVE_AND_TURN, alliance), clearance);
    planner.update(
        RobotStateSnapshot.measured(
            0, CompAutoField.pose(TURN_START, alliance), new ChassisSpeeds(), true));
  }

  TrailblazerTrenchRotationAuto(Trailblazer trailblazer, Alliance alliance) {
    super(State.CLEAR_WALL, trailblazer, alliance);
  }

  @Override
  protected void afterTransition(State state) {
    switch (state) {
      case CLEAR_WALL -> activate(CLEAR_WALL);
      case DRIVE_AND_TURN -> activate(DRIVE_AND_TURN);
      case DONE -> {}
    }
  }

  @Override
  protected State getNextState(State state) {
    if (!trailblazer.atGoal()) {
      return state;
    }
    return switch (state) {
      case CLEAR_WALL -> State.DRIVE_AND_TURN;
      case DRIVE_AND_TURN, DONE -> State.DONE;
    };
  }
}
