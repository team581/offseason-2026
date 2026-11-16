package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.shooter.ShooterConfig;

/**
 * Three-cycle right-trench autonomous: sweeps neutral zone fuel across three loops through the
 * right trench and returns over the right bump to score into the Hub.
 */
public final class RightTrenchAuto extends TrailblazerAuto<RightTrenchAuto.State> {
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

  // RightNormalAuto in the TB1 archive was authored on red. Store its nominal route in
  // the blue frame expected by TrailblazerAuto, then let the normal alliance flip handle red.
  private static final double RED_TRENCH_Y = FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY();
  private static final double RED_BUMP_Y = FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() - 0.13;
  private static final double RED_SHOOT_X = 13.576;
  private static final Translation2d SHOOT_POINT =
      bluePose(RED_SHOOT_X, RED_BUMP_Y, 0).getTranslation();

  static final Pose2d START = bluePose(12.1, 7.6, -90);

  static final AutoSegment CYCLE_1_SEGMENT =
      finishCycle(
          Trailblazer.segment("right-trench-cycle-1")
              .through(bluePose(10.489, RED_TRENCH_Y, -90), 0.75)
              .through(bluePose(8.744, 6.049, -115), 0.30)
              .through(bluePose(8.316, 4.945, -115), 0.35)
              .mark(Marker.PRIORITIZE_INTAKE)
              .through(bluePose(8.856, 4.445, -40), 0.20)
              .through(bluePose(9.356, 4.245, 0), 0.20)
              .through(bluePoint(9.756, 4.545), 0.30)
              .through(bluePose(10.15, RED_BUMP_Y, 0), 0.20)
              .through(bluePose(10.6, RED_BUMP_Y, 0), 0.30)
              .mark(Marker.PREPARE_BUMP));

  static final AutoSegment CYCLE_2_SEGMENT =
      finishCycle(
          returnToTrench("right-trench-cycle-2", 0.30)
              .through(bluePose(10.628, 7.2, -140), 0.40)
              .through(bluePose(9.140, 5.653, -130), 0.40)
              .through(bluePose(8.500, 4.81, -120), 0.40)
              .through(bluePoint(8.456, 4.6), 0.20)
              .through(bluePoint(9.156, 4.445), 0.20)
              .through(bluePoint(9.556, 4.445), 0.20)
              .through(bluePoint(10.356, 4.045), 0.30)
              .through(bluePoint(9.556, 5.145), 0.20)
              .through(bluePose(10.5, RED_BUMP_Y, 0), 0.20)
              .mark(Marker.PREPARE_BUMP));

  static final AutoSegment CYCLE_3_SEGMENT =
      finishCycle(
          returnToTrench("right-trench-cycle-3", 0.20)
              .through(bluePose(9.6, 6.353, -130), 0.40)
              .through(bluePose(8.7, 5.593, -90), 0.30)
              .through(bluePose(8.600, 4.31, -90), 0.30)
              .through(bluePoint(9.31, 4.3), 0.30)
              .through(bluePose(9.7, RED_BUMP_Y, 0), 0.30)
              .through(bluePose(10.6, RED_BUMP_Y, 0), 0.30)
              .mark(Marker.PREPARE_BUMP));

  /** Converts an archived red-alliance field point in meters to the blue authoring frame. */
  private static Translation2d bluePoint(double redX, double redY) {
    return bluePose(redX, redY, 0).getTranslation();
  }

  /** Converts a red-alliance pose in meters and degrees to the blue authoring frame. */
  private static Pose2d bluePose(double redX, double redY, double redDegrees) {
    return CompAutoField.FIELD.flipPose(CompAutoField.pose(redX, redY, redDegrees));
  }

  /** Adds the bump crossing, hub-facing shot stop, and shared angular limits to a cycle. */
  private static AutoSegment finishCycle(AutoSegment.Builder builder) {
    return builder
        .withLimits(
            MotionLimits.robotMaximum()
                .withMaxAngularSpeedDegrees(210)
                .withMaxAngularAccelerationDegrees(320))
        .through(
            bluePose(13.25, RED_BUMP_Y, 0),
            0.25,
            goal -> goal.heading(HeadingPlan.to(Rotation2d.k180deg).before(0.25)))
        .stopAt(SHOOT_POINT, Goal::faceHub)
        .build();
  }

  /** Starts another intake cycle with the shared trench entry, intake marker, and exit guide. */
  private static AutoSegment.Builder returnToTrench(String name, double exitWithin) {
    // The archive repeats 13.2 m for its 180° heading. Use a single 160° entry point
    // and approach the 179° trench heading on the next leg.
    return Trailblazer.segment(name)
        .through(bluePose(13.2, RED_TRENCH_Y, 160), 0.30)
        .through(bluePose(12.5, RED_TRENCH_Y, 179), 0.30)
        .mark(Marker.START_INTAKE)
        .through(bluePose(11.5, RED_TRENCH_Y, 179), exitWithin)
        // Keep the bumper parallel to the trench until it clears the neutral-side post.
        .through(
            bluePose(10.83, RED_TRENCH_Y, 179),
            0.04,
            goal -> goal.heading(HeadingPlan.to(Rotation2d.fromDegrees(-1)).after(0.15)));
  }

  /** Returns the stop pose with the rear shooter aimed at the hub on the selected alliance. */
  static Pose2d shootPose(Alliance alliance) {
    return CompAutoField.pose(
        new Pose2d(
            SHOOT_POINT,
            ShooterConfig.calculateAimingAngle(SHOOT_POINT, FieldUtil.BLUE_HUB_TRANSLATION)),
        alliance);
  }

  /** Creates the three-cycle right-trench state machine for the given robot context. */
  public RightTrenchAuto(AutoContext context) {
    super(State.INTAKE_CYCLE_1, START, context, CYCLE_1_SEGMENT, CYCLE_2_SEGMENT, CYCLE_3_SEGMENT);
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
      case SHOOT_1, SHOOT_2, SHOOT_3 -> {
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
      case INTAKE_CYCLE_1 -> atGoal() ? State.SHOOT_1 : currentState;
      case SHOOT_1 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_2 : currentState;
      case INTAKE_CYCLE_2 -> atGoal() ? State.SHOOT_2 : currentState;
      case SHOOT_2 -> AutoActions.finishedShooting(this) ? State.INTAKE_CYCLE_3 : currentState;
      case INTAKE_CYCLE_3 -> atGoal() ? State.SHOOT_3 : currentState;
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
          AutoActions.prepareForBumpShot(robot, SHOOT_POINT);
        }
      }
      case INTAKE_CYCLE_2, INTAKE_CYCLE_3 -> {
        if (passedMarker(Marker.PREPARE_BUMP)) {
          AutoActions.prepareForBumpShot(robot, SHOOT_POINT);
        } else if (passedMarker(Marker.START_INTAKE)) {
          robot.intakeAutoRequest();
        }
      }
      case SHOOT_1, SHOOT_2, SHOOT_3 -> robot.prepareScoreRequest();
      case DONE -> {}
    }
  }
}
