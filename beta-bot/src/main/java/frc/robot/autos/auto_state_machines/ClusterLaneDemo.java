package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.BaseImperativeAuto;
// Make sure to create this enum in your auto_state package!
import frc.robot.autos.auto_state_machines.auto_state.ClusterLaneDemoState;
import frc.robot.cluster_map.Lane;
import frc.robot.robot_manager.RobotManager;

public class ClusterLaneDemo extends BaseImperativeAuto<ClusterLaneDemoState> {

  // initial path to get out of the trench and look at the field
  private final AutoSegment driveThroughTrench =
      Trailblazer.segment(
              AutoPoint.ofRed(
                      new Pose2d(
                          11.5,
                          FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(),
                          Rotation2d.fromDegrees(180 + 50)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.3)),
              AutoPoint.ofRed(new Pose2d(10.5, 7.424, Rotation2d.fromDegrees(-114.56)))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 50)))
          .withLinearConstraints(4.0, 8)
          .withAngularConstraints(Units.rotationsToRadians(5), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.5, 50));

  private final AutoSegment didNotSee =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.0, 6.8, Rotation2d.k180deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)))
          .withLinearConstraints(2.0, 5)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment lane2Segment =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(9.0, 7.24, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)),
              AutoPoint.ofRed(new Pose2d(9.0, 3.0, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment lane1Segment =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.0, 7.24, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)),
              AutoPoint.ofRed(new Pose2d(10.0, 3.0, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  private final AutoSegment lane0Segment =
      Trailblazer.segment(
              AutoPoint.ofRed(new Pose2d(10.2, 7.24, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)),
              AutoPoint.ofRed(new Pose2d(10.5, 3.0, Rotation2d.kCW_90deg))
                  .withTransitionTolerance(new PoseErrorTolerance(0.2, 30)))
          .withLinearConstraints(4.5, 8)
          .withAngularConstraints(Units.rotationsToRadians(4), Units.rotationsToRadians(4))
          .untilFinished(new PoseErrorTolerance(0.2, 3));

  public ClusterLaneDemo(RobotManager robotManager, Trailblazer trailblazer) {
    super(ClusterLaneDemoState.DRIVE_THROUGH_TRENCH, robotManager, trailblazer);
  }

  @Override
  public Point getStartingPoint() {
    return Point.ofRed(
        new Pose2d(12.1, FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY(), Rotation2d.k180deg));
  }

  @Override
  protected ClusterLaneDemoState getNextState(ClusterLaneDemoState currentState) {
    return switch (currentState) {
      case DRIVE_THROUGH_TRENCH -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield ClusterLaneDemoState.CHOOSE_LANE;
        } else {
          yield currentState;
        }
      }
      case CHOOSE_LANE -> {
        yield ClusterLaneDemoState.EXECUTE_LANE_PATH;
      }
      case EXECUTE_LANE_PATH -> {
        if (trailblazer.atGoal(robotManager.localization.getPose())) {
          yield ClusterLaneDemoState.DONE;
        } else {
          yield currentState;
        }
      }
      case DONE -> currentState;
    };
  }

  @Override
  protected void whileInState(ClusterLaneDemoState newState) {
    switch (newState) {
      case DRIVE_THROUGH_TRENCH -> {
        trailblazer.setActiveSegment(driveThroughTrench);
      }
      case CHOOSE_LANE -> {}
      case EXECUTE_LANE_PATH -> {
        robotManager.intakeAutoRequest();
      }
      case DONE -> {
        robotManager.cancelIntakeRequest();
      }
    }
  }

  @Override
  protected void afterTransition(ClusterLaneDemoState newState) {
    switch (newState) {
      case DRIVE_THROUGH_TRENCH -> {
        robotManager.homeDeployInAutoRequest();
      }
      case CHOOSE_LANE -> {}

      case EXECUTE_LANE_PATH -> {
        Lane bestLane = robotManager.clusterMap.getBestClusterLane();
        switch (bestLane) {
          case LANE_0 -> trailblazer.setActiveSegment(lane0Segment);
          case LANE_1 -> trailblazer.setActiveSegment(lane1Segment);
          case LANE_2 -> trailblazer.setActiveSegment(lane2Segment);
          case TRENCH ->
              // TODO: make a trench path
              trailblazer.setActiveSegment(didNotSee);
          default -> {
            trailblazer.setActiveSegment(didNotSee);
          }
        }
      }
      case DONE -> {}
    }
  }
}
