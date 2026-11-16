package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
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

  private static final double TRENCH_Y = CompAutoField.RIGHT_TRENCH_Y_METERS;
  private static final double BUMP_LANE_Y = CompAutoField.RIGHT_BUMP_CENTER_Y_METERS + 0.10;
  private static final double BUMP_CREST_X = 4.65;
  private static final double SHOOT_X = 3.35;
  private static final Translation2d SHOOT_POINT = new Translation2d(SHOOT_X, BUMP_LANE_Y);

  static final Pose2d START = CompAutoField.pose(4.441, 0.469, 90.0);

  static final AutoSegment CYCLE_1_SEGMENT =
      Trailblazer.segment("right-trench-cycle-1")
          .through(6.200, TRENCH_Y, 0.65, g -> g.faceDegrees(75.0))
          .through(8.150, 2.000, 0.95, g -> g.faceDegrees(90.0).mark(Marker.PRIORITIZE_INTAKE))
          .through(7.800, 3.450, 0.95, g -> g.faceDegrees(165.0))
          .through(6.000, BUMP_LANE_Y, 0.65, g -> g.faceDegrees(180.0).mark(Marker.PREPARE_BUMP))
          .through(BUMP_CREST_X, BUMP_LANE_Y, g -> g.faceDegrees(180.0))
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_2_SEGMENT =
      Trailblazer.segment("right-trench-cycle-2")
          .through(3.120, 0.900, 0.35)
          .through(3.850, TRENCH_Y, 0.22, g -> g.faceDegrees(0.0).mark(Marker.START_INTAKE))
          .through(6.200, TRENCH_Y, 0.40, g -> g.faceDegrees(0.0))
          .through(8.000, 1.950, 0.95, g -> g.faceDegrees(90.0))
          .through(7.650, 3.450, 0.95, g -> g.faceDegrees(165.0))
          .through(6.000, BUMP_LANE_Y, 0.65, g -> g.faceDegrees(180.0).mark(Marker.PREPARE_BUMP))
          .through(BUMP_CREST_X, BUMP_LANE_Y, g -> g.faceDegrees(180.0))
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static final AutoSegment CYCLE_3_SEGMENT =
      Trailblazer.segment("right-trench-cycle-3")
          .through(3.120, 0.900, 0.35)
          .through(3.850, TRENCH_Y, 0.22, g -> g.faceDegrees(0.0).mark(Marker.START_INTAKE))
          .through(6.150, TRENCH_Y, 0.35, g -> g.faceDegrees(0.0))
          .through(7.700, 1.900, 0.95, g -> g.faceDegrees(90.0))
          .through(7.450, 3.350, 0.95, g -> g.faceDegrees(165.0))
          .through(6.000, BUMP_LANE_Y, 0.65, g -> g.faceDegrees(180.0).mark(Marker.PREPARE_BUMP))
          .through(BUMP_CREST_X, BUMP_LANE_Y, g -> g.faceDegrees(180.0))
          .stopAt(SHOOT_POINT, Goal::faceHub)
          .build();

  static Pose2d shootPose(Alliance alliance) {
    return CompAutoField.pose(
        new Pose2d(
            SHOOT_POINT,
            ShooterConfig.calculateAimingAngle(SHOOT_POINT, FieldUtil.BLUE_HUB_TRANSLATION)),
        alliance);
  }

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
