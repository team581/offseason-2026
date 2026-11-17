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

  private static final Pose2d BUMP_LANDING = bluePose(13.25, RED_BUMP_Y, 0);

  static final Pose2d START = bluePose(12.1, 7.6, -90);

  private static final MotionLimits CYCLE_LIMITS =
      MotionLimits.robotMaximum()
          .withMaxAngularSpeedDegrees(210)
          .withMaxAngularAccelerationDegrees(320);
  static final AutoSegment CYCLE_1_SEGMENT =
      Trailblazer.segment("right-trench-cycle-1")
          .withLimits(CYCLE_LIMITS)
          // Hold the launch heading through the wall-side opening.
          .through(bluePose(10.089, RED_TRENCH_Y, -90), 0.25, 15)
          .through(bluePose(8.744, 6.049, -115), 0.30, 15)
          .through(bluePose(8.716, 4.945, -90), 0.55, 35)
          .mark(Marker.PRIORITIZE_INTAKE)
          .through(bluePose(8.856, 4.445, -40), 0.20, 30)
          .through(bluePose(9.356, 4.245, 0), 0.20, 35)
          .through(bluePose(9.756, 4.545, 35), 0.30, 15)
          .through(bluePose(10.15, RED_BUMP_Y, 0), 0.20, 15)
          .through(bluePose(10.6, RED_BUMP_Y, 0), 0.30, 5)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, RightTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_2_SEGMENT =
      Trailblazer.segment("right-trench-cycle-2")
          .withLimits(CYCLE_LIMITS)
          .through(bluePose(13.2, RED_TRENCH_Y, 175), 0.35, 15)
          .through(bluePose(12.5, RED_TRENCH_Y, 179), 0.20, 5)
          .mark(Marker.START_INTAKE)
          .through(bluePose(10.9, RED_TRENCH_Y, 179), 0.30, 15)
          .through(bluePose(10.228, 6.8, -140), 0.40, 10)
          .through(bluePose(9.040, 5.553, -105), 0.40, 10)
          .through(bluePose(8.800, 4.81, -90), 0.50, 25)
          .through(bluePose(9.356, 4.255, -10), 0.40, 15)
          .through(bluePose(10.556, 4.045, 0), 0.20, 15)
          .through(bluePose(9.556, 5.145, 15), 0.20, 25)
          .through(bluePose(10.5, RED_BUMP_Y, 0), 0.20, 25)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, RightTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_3_SEGMENT =
      Trailblazer.segment("right-trench-cycle-3")
          .withLimits(CYCLE_LIMITS)
          .through(bluePose(13.2, RED_TRENCH_Y, 175), 0.35, 15)
          .through(bluePose(12.5, RED_TRENCH_Y, 179), 0.20, 5)
          .mark(Marker.START_INTAKE)
          .through(bluePose(10.9, RED_TRENCH_Y, 179), 0.20, 15)
          .through(bluePose(9.6, 6.353, -130), 0.40, 25)
          .through(bluePose(8.7, 5.593, -90), 0.30, 25)
          .through(bluePose(8.600, 4.31, -70), 0.30, 25)
          .through(bluePose(9.31, 4.3, 32), 0.30, 25)
          .through(bluePose(9.7, RED_BUMP_Y, 0), 0.30, 25)
          .through(bluePose(10.6, RED_BUMP_Y, 0), 0.30, 25)
          .mark(Marker.PREPARE_BUMP)
          .through(BUMP_LANDING, 0.25, RightTrenchAuto::holdBumpHeading)
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  /** Converts a red-alliance pose in meters and degrees to the blue authoring frame. */
  private static Pose2d bluePose(double redX, double redY, double redDegrees) {
    return CompAutoField.FIELD.flipPose(CompAutoField.pose(redX, redY, redDegrees));
  }

  private static Goal holdBumpHeading(Goal goal) {
    return goal.heading(
        HeadingPlan.to(Rotation2d.k180deg).before(0.25).tolerance(Rotation2d.fromDegrees(12)));
  }

  /** Returns the stop pose with the rear shooter aimed at the hub on the selected alliance. */
  static Pose2d shootPose(Alliance alliance) {
    return CompAutoField.pose(
        new Pose2d(
            SHOOT_POINT,
            ShooterConfig.calculateAimingAngle(SHOOT_POINT, FieldUtil.BLUE_HUB_TRANSLATION)),
        alliance);
  }

  private boolean preparingBump;

  /** Creates the three-cycle right-trench state machine for the given robot context. */
  public RightTrenchAuto(AutoContext context) {
    super(State.INTAKE_CYCLE_1, START, context, CYCLE_1_SEGMENT, CYCLE_2_SEGMENT, CYCLE_3_SEGMENT);
  }

  private void prepareForBump() {
    if (!preparingBump) {
      preparingBump = true;
      // The tracker projects body-up tilt. Use the uphill sign from the legacy bump autos.
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
