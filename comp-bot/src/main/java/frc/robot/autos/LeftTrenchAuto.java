package frc.robot.autos;

import static frc.robot.config.CompAutoField.fromRed;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Three-cycle left-trench autonomous: sweeps neutral zone fuel across three loops through the left
 * trench and returns over the left bump to score into the Hub.
 */
public final class LeftTrenchAuto extends TrailblazerAuto<LeftTrenchAuto.State> {
  public enum Marker {
    PRIORITIZE_INTAKE,
    START_INTAKE,
    PREPARE_BUMP
  }

  public enum State {
    INTAKE_CYCLE_1,
    CROSS_BUMP_TO_SHOOT_1,
    SHOOT_1,
    INTAKE_CYCLE_2,
    CROSS_BUMP_TO_SHOOT_2,
    SHOOT_2,
    INTAKE_CYCLE_3,
    CROSS_BUMP_TO_SHOOT_3,
    SHOOT_3
  }

  private static final double RED_TRENCH_Y = FieldUtil.RED_DEPOT_TRENCH_CENTER.getY();
  private static final double RED_BUMP_Y = FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.13;
  private static final double RED_SHOOT_X = 13.576;
  private static final Translation2d SHOOT_POINT =
      fromRed(RED_SHOOT_X, RED_BUMP_Y, 0).getTranslation();

  private static final Pose2d BUMP_LANDING = fromRed(13.16, RED_BUMP_Y, 0);

  static final Pose2d START = fromRed(12.1, 0.469, 90);

  private static final MotionLimits CYCLE_LIMITS =
      MotionLimits.robotMaximum()
          .withMaxAngularSpeedDegrees(210)
          .withMaxAngularAccelerationDegrees(320);

  static final AutoSegment CYCLE_1_SEGMENT =
      Trailblazer.segment("left-trench-cycle-1")
          .withLimits(CYCLE_LIMITS)
          .through(fromRed(10.089, RED_TRENCH_Y, 90), 0.15, 15)
          .through(fromRed(8.744, 2.020, 115), 0.30, 5)
          .through(fromRed(8.016, 3.124, 115), 0.55, 35)
          .mark(Marker.PRIORITIZE_INTAKE)
          .through(fromRed(8.856, 3.624, 40), 0.20, 30)
          .through(fromRed(9.356, 3.824, 0), 0.20, 35)
          .through(fromRed(9.756, 3.524, -35), 0.30, 15)
          .through(fromRed(10.15, RED_BUMP_Y, 0), 0.20, 15)
          .through(fromRed(10.6, RED_BUMP_Y, 0), 0.30, 5)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, LeftTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_2_SEGMENT =
      Trailblazer.segment("left-trench-cycle-2")
          .withLimits(CYCLE_LIMITS)
          .through(fromRed(13.2, RED_TRENCH_Y, -175), 0.35, 15)
          .through(fromRed(12.5, RED_TRENCH_Y, -179), 0.20, 5)
          .mark(Marker.START_INTAKE)
          .through(fromRed(10.9, RED_TRENCH_Y, -179), 0.30, 15)
          .through(fromRed(10.228, 1.269, 140), 0.40, 10)
          .through(fromRed(9.040, 2.516, 135), 0.40, 10)
          .through(fromRed(8.300, 3.259, 100), 0.50, 25)
          .through(fromRed(9.156, 3.814, 10), 0.40, 15)
          .through(fromRed(10.556, 4.024, 0), 0.20, 15)
          .through(fromRed(9.556, 2.924, -15), 0.20, 25)
          .through(fromRed(10.5, RED_BUMP_Y, 0), 0.20, 25)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, LeftTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_3_SEGMENT =
      Trailblazer.segment("left-trench-cycle-3")
          .withLimits(CYCLE_LIMITS)
          .through(fromRed(13.2, RED_TRENCH_Y, -175), 0.35, 15)
          .through(fromRed(12.5, RED_TRENCH_Y, -179), 0.20, 5)
          .mark(Marker.START_INTAKE)
          .through(fromRed(10.9, RED_TRENCH_Y, -179), 0.20, 15)
          .through(fromRed(9.6, 1.716, 130), 0.40, 25)
          .through(fromRed(8.7, 2.476, 90), 0.30, 25)
          .through(fromRed(8.600, 3.759, 70), 0.30, 25)
          .through(fromRed(9.31, 3.769, -32), 0.30, 25)
          .through(fromRed(9.7, RED_BUMP_Y, 0), 0.30, 25)
          .through(fromRed(10.6, RED_BUMP_Y, 0), 0.30, 25)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, LeftTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  private static Goal holdBumpHeading(Goal goal) {
    return goal.heading(
        HeadingPlan.to(Rotation2d.k180deg).before(0.25).tolerance(Rotation2d.fromDegrees(12)));
  }

  private final AutoSegment scoreSegment;

  public LeftTrenchAuto(AutoContext context) {
    super(State.INTAKE_CYCLE_1, START, context, CYCLE_1_SEGMENT, CYCLE_2_SEGMENT, CYCLE_3_SEGMENT);
    scoreSegment = AutoActions.scoreSegment(CYCLE_1_SEGMENT);
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case INTAKE_CYCLE_1 -> {
        drive(CYCLE_1_SEGMENT);
        AutoActions.startFirstIntakeCycle(robot);
      }
      case INTAKE_CYCLE_2 -> {
        drive(CYCLE_2_SEGMENT);
        AutoActions.startNextCycle(robot);
      }
      case INTAKE_CYCLE_3 -> {
        drive(CYCLE_3_SEGMENT);
        AutoActions.startNextCycle(robot);
      }
      case CROSS_BUMP_TO_SHOOT_1 -> {
        robot.powerManager.idleRequest();
        robot.cancelIntakeRequest();
        AutoActions.requestBumpCrossing(this, BUMP_LANDING);
      }
      case CROSS_BUMP_TO_SHOOT_2 -> AutoActions.requestBumpCrossing(this, BUMP_LANDING);
      case CROSS_BUMP_TO_SHOOT_3 -> {
        robot.idleRequest();
        AutoActions.requestBumpCrossing(this, BUMP_LANDING);
      }
      case SHOOT_1 -> stopDriving();
      case SHOOT_2, SHOOT_3 -> {
        robot.cancelIntakeRequest();
        stopDriving();
      }
    }
  }

  @Override
  protected State getNextState(State currentState) {
    return switch (currentState) {
      case INTAKE_CYCLE_1 ->
          passedMarker(Marker.PREPARE_BUMP) ? State.CROSS_BUMP_TO_SHOOT_1 : currentState;
      case CROSS_BUMP_TO_SHOOT_1 ->
          AutoActions.finishedBumpCrossing(robot) ? State.SHOOT_1 : currentState;
      case SHOOT_1 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_2 : currentState;
      case INTAKE_CYCLE_2 ->
          passedMarker(Marker.PREPARE_BUMP) ? State.CROSS_BUMP_TO_SHOOT_2 : currentState;
      case CROSS_BUMP_TO_SHOOT_2 ->
          AutoActions.finishedBumpCrossing(robot) ? State.SHOOT_2 : currentState;
      case SHOOT_2 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_3 : currentState;
      case INTAKE_CYCLE_3 ->
          passedMarker(Marker.PREPARE_BUMP) ? State.CROSS_BUMP_TO_SHOOT_3 : currentState;
      case CROSS_BUMP_TO_SHOOT_3 ->
          AutoActions.finishedBumpCrossing(robot) ? State.SHOOT_3 : currentState;
      case SHOOT_3 -> currentState;
    };
  }

  @Override
  protected void whileInState(State state) {
    switch (state) {
      case INTAKE_CYCLE_1 -> {
        if (passedMarker(Marker.PRIORITIZE_INTAKE)) {
          robot.powerManager.prioritizeIntakeRequest();
        }
      }
      case INTAKE_CYCLE_2, INTAKE_CYCLE_3 -> {
        if (passedMarker(Marker.START_INTAKE)) {
          robot.intakeAutoRequest();
        }
      }
      case CROSS_BUMP_TO_SHOOT_1, CROSS_BUMP_TO_SHOOT_2, CROSS_BUMP_TO_SHOOT_3 ->
          AutoActions.whileCrossingBump(this, BUMP_LANDING);
      case SHOOT_1 -> {
        AutoActions.scoreInPlace(this, scoreSegment);
        prepareNext(CYCLE_2_SEGMENT);
      }
      case SHOOT_2 -> {
        AutoActions.scoreInPlace(this, scoreSegment);
        prepareNext(CYCLE_3_SEGMENT);
      }
      case SHOOT_3 -> AutoActions.scoreInPlace(this, scoreSegment);
    }
  }
}
