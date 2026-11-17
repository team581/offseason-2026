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
    SHOOT_1,
    INTAKE_CYCLE_2,
    SHOOT_2,
    INTAKE_CYCLE_3,
    SHOOT_3,
    DONE
  }

  private static final double RED_TRENCH_Y = FieldUtil.RED_DEPOT_TRENCH_CENTER.getY();
  private static final double RED_BUMP_Y = FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.13;
  private static final double RED_SHOOT_X = 13.576;
  private static final Translation2d SHOOT_POINT =
      fromRed(RED_SHOOT_X, RED_BUMP_Y, 0).getTranslation();

  private static final Pose2d BUMP_LANDING = fromRed(13.25, RED_BUMP_Y, 0);

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

  private boolean preparingBump;

  public LeftTrenchAuto(AutoContext context) {
    super(State.INTAKE_CYCLE_1, START, context, CYCLE_1_SEGMENT, CYCLE_2_SEGMENT, CYCLE_3_SEGMENT);
  }

  private void prepareForBump() {
    if (!preparingBump) {
      preparingBump = true;

      var tiltDirection = fieldPose(BUMP_LANDING).getRotation().plus(Rotation2d.k180deg);
      robot.localization.imu.bumpCrossingTracker.trackCrossing(tiltDirection);
    }
    AutoActions.prepareForBumpShot(robot, SHOOT_POINT);
  }

  private boolean readyToShoot() {
    return atGoal() && !robot.localization.imu.bumpCrossingTracker.isOnBump();
  }

  @Override
  protected void afterTransition(State newState) {
    preparingBump = false;
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
      case SHOOT_1, SHOOT_2, SHOOT_3 -> {
        robot.localization.imu.bumpCrossingTracker.cancelCrossing();
        stopDriving();
        AutoActions.startScoring(robot);
      }
      case DONE -> {
        stopDriving();
        robot.idleRequest();
      }
    }
  }

  @Override
  protected State getNextState(State currentState) {
    return switch (currentState) {
      case INTAKE_CYCLE_1 -> readyToShoot() ? State.SHOOT_1 : currentState;
      case SHOOT_1 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_2 : currentState;
      case INTAKE_CYCLE_2 -> readyToShoot() ? State.SHOOT_2 : currentState;
      case SHOOT_2 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_3 : currentState;
      case INTAKE_CYCLE_3 -> readyToShoot() ? State.SHOOT_3 : currentState;
      case SHOOT_3 -> AutoActions.finishedShooting(this) ? State.DONE : currentState;
      case DONE -> currentState;
    };
  }

  @Override
  protected void whileInState(State state) {
    switch (state) {
      case INTAKE_CYCLE_1 -> {
        if (passedMarker(Marker.PRIORITIZE_INTAKE)) {
          robot.powerManager.prioritizeIntakeRequest();
        }
        if (passedMarker(Marker.PREPARE_BUMP)) {
          prepareForBump();
        }
      }
      case INTAKE_CYCLE_2, INTAKE_CYCLE_3 -> {
        if (passedMarker(Marker.PREPARE_BUMP)) {
          prepareForBump();
        } else if (passedMarker(Marker.START_INTAKE)) {
          robot.intakeAutoRequest();
        }
      }
      case SHOOT_1, SHOOT_2, SHOOT_3 -> robot.prepareScoreRequest();
      case DONE -> {}
    }
  }
}
