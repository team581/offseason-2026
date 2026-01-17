package frc.robot.robot_manager;

import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private final Intake intake;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Swerve swerve;
  private final Vision vision;
  private final Localization localization;

  public RobotManager(
      Intake intake,
      Shooter shooter,
      Feeder feeder,
      Swerve swerve,
      Vision vision,
      Localization localization) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.intake = intake;
    this.shooter = shooter;
    this.feeder = feeder;
    this.swerve = swerve;
    this.vision = vision;
    this.localization = localization;

    DogLog.log("Robot/StateCount", RobotState.values().length);
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case PREPARE_SHOOT_HUB -> {
        if (shooter.atGoal() && vision.seeingTag() && FieldUtil.isRobotInAllianceZone(robotPose)) {
          yield RobotState.SHOOT_HUB;
        }
        yield currentState;
      }
      case PREPARE_FEED_1 -> shooter.atGoal() ? RobotState.FEED_1 : currentState;
      case PREPARE_FEED_2 -> shooter.atGoal() ? RobotState.FEED_2 : currentState;
      case SHOOT_HUB ->
          !FieldUtil.isRobotInAllianceZone(robotPose) ? RobotState.IDLE : currentState;
      default -> currentState;
    };
  }

  private Pose2d robotPose = new Pose2d();
  private double distanceToHub = 0;
  private double angleToHub = 0;

  private double distanceToFeed = 0;
  private double angleToFeed = 0;

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        shooter.idleRequest();
        feeder.idleRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
      }
      case PREPARE_FORCE_SHOOT -> {
          shooter.scoreRequest();
          swerve.normalDriveRequest();
      }
      case FORCE_SHOOT -> {
          shooter.scoreRequest();
          intake.intakeRequest();
          feeder.feedRequest();
          swerve.normalDriveRequest();
      }
      case WAIT_FEED_1, PREPARE_FEED_1 -> {
        shooter.feedRequest();
        swerve.snapsDriveRequest(SnapUtil.getFeed1Angle(FmsUtil.isRedAlliance()));
      }
      case WAIT_FEED_2, PREPARE_FEED_2 -> {
        shooter.feedRequest();
        swerve.snapsDriveRequest(SnapUtil.getFeed2Angle(FmsUtil.isRedAlliance()));
      }
      case FEED_1 -> {
        shooter.feedRequest();
        feeder.feedRequest();
        intakeRequest();
        swerve.snapsDriveRequest(SnapUtil.getFeed1Angle(FmsUtil.isRedAlliance()));
      }
      case FEED_2 -> {
        shooter.feedRequest();
        feeder.feedRequest();
        intakeRequest();
        swerve.snapsDriveRequest(SnapUtil.getFeed2Angle(FmsUtil.isRedAlliance()));
      }
      case WAIT_SHOOT_HUB, PREPARE_SHOOT_HUB -> {
        shooter.scoreRequest();
        swerve.hubAimRequest(angleToHub);
      }
      case SHOOT_HUB -> {
        shooter.scoreRequest();
        feeder.feedRequest();
        intakeRequest();
        swerve.hubAimRequest(angleToHub);
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    // TODO: distance
    shooter.setHubDistance(distanceToHub);
    shooter.setFeedDistance(distanceToFeed);

    switch (state) {
      case WAIT_FEED_1, PREPARE_FEED_1, FEED_1 -> {
        swerve.snapsDriveRequest(angleToFeed);
      }
      case WAIT_FEED_2, PREPARE_FEED_2, FEED_2 -> swerve.snapsDriveRequest(angleToFeed);
      case WAIT_SHOOT_HUB, PREPARE_SHOOT_HUB, SHOOT_HUB -> {
        if (FieldUtil.isRobotInAllianceZone(robotPose)) {
          swerve.hubAimRequest(angleToHub);
        } else {
          swerve.snapsDriveRequest(angleToHub);
        }
      }
      default -> swerve.normalDriveRequest();
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    var hubPose = FieldUtil.getHubPose();
    distanceToHub = robotPose.getTranslation().getDistance(hubPose.getTranslation());
    angleToFeed =
        angleToHub =
            robotPose.relativeTo(FieldUtil.getHubPose()).getTranslation().getAngle().getDegrees();

    var feedPose = FieldUtil.RED_FEED_POSE;
    distanceToFeed = robotPose.getTranslation().getDistance(feedPose.getTranslation());
    angleToFeed = robotPose.relativeTo(feedPose).getTranslation().getAngle().getDegrees();
  }

  private void setStateFailSafe(RobotState newState) {
    if (getState().climbingOrRehoming()) {
      return;
    }
    setStateFromRequest(newState);
  }

  public void idleRequest() {
    setStateFailSafe(RobotState.IDLE);
  }

  public void intakeRequest() {
    intake.intakeRequest();
  }

  public void forceShootRequest() {
    setStateFailSafe(RobotState.PREPARE_FORCE_SHOOT);
  }

  public void cancelIntakeRequest() {
    intake.idleRequest();
  }

  public void shootHubWaitRequest() {
    setStateFailSafe(RobotState.WAIT_SHOOT_HUB);
  }

  public void feed1WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_1);
  }

  public void feed2WaitRequest() {
    setStateFailSafe(RobotState.WAIT_FEED_2);
  }

  public void toggleHubRequest() {
    switch (getState()) {
      case PREPARE_SHOOT_HUB, SHOOT_HUB -> setStateFailSafe(RobotState.IDLE);
      default -> setStateFailSafe(RobotState.PREPARE_SHOOT_HUB);
    }
  }

  public void toggleFeedRequest() {
    switch (getState()) {
      case WAIT_FEED_2 -> setStateFailSafe(RobotState.PREPARE_FEED_2);
      case PREPARE_FEED_1, PREPARE_FEED_2, FEED_1, FEED_2 -> setStateFailSafe(RobotState.IDLE);
      default -> setStateFailSafe(RobotState.PREPARE_FEED_1);
    }
  }

  public void climbSequenceForward() {
    cancelIntakeRequest();
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1);
      case CLIMB_1_LINEUP_L1 -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1);
      case CLIMB_2_RAISING_L1 -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1);
      case CLIMB_3_HANGING_L1 -> setStateFromRequest(RobotState.CLIMB_4_RAISING_L2);

      case CLIMB_4_RAISING_L2 -> setStateFromRequest(RobotState.CLIMB_5_HANGING_L2);
      case CLIMB_5_HANGING_L2 -> setStateFromRequest(RobotState.CLIMB_6_RAISING_L3);

      case CLIMB_6_RAISING_L3 -> setStateFromRequest(RobotState.CLIMB_7_HANGING_L3);
      case CLIMB_7_HANGING_L3 -> {}
    }
  }

  public void climbSequenceForwardAuto() {
    cancelIntakeRequest();
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTO);
      case CLIMB_1_LINEUP_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTO);
      case CLIMB_2_RAISING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1_AUTO);
      case CLIMB_3_HANGING_L1_AUTO -> {}
    }
  }

  public void climbSequenceBackward() {
    cancelIntakeRequest();
    switch (getState()) {
      default -> {}
      case CLIMB_1_LINEUP_L1 -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1 -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1);
      case CLIMB_3_HANGING_L1 -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1);

      case CLIMB_1_LINEUP_L1_AUTO -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTO);
      case CLIMB_3_HANGING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTO);

      case CLIMB_4_RAISING_L2 -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1);
      case CLIMB_5_HANGING_L2 -> setStateFromRequest(RobotState.CLIMB_4_RAISING_L2);

      case CLIMB_6_RAISING_L3 -> setStateFromRequest(RobotState.CLIMB_5_HANGING_L2);
      case CLIMB_7_HANGING_L3 -> setStateFromRequest(RobotState.CLIMB_6_RAISING_L3);
    }
  }
}
