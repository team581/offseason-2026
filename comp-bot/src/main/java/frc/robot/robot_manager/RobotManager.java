package frc.robot.robot_manager;

import java.lang.reflect.Field;

import com.team581.math.MathHelpers;
import com.team581.math.SwerveAssist;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.lights.LightsState;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  private final ShooterHood shooterHood;
  private final Shooter shooter;
  private final DyeRotor dyeRotor;
  private final Deploy deploy;
  private final Turret turret;
  private final Intake intake;
  private final Vision vision;
  private final Lights lights;

  private FeedLocation feedLocation = FeedLocation.LEFT;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private double hubGoalAngle = 0.0;
  private double hubDistance = 0.0;

  private double feedLeftGoalAngle = 0.0;
  private double feedLeftDistance = 0.0;

  private double feedRightGoalAngle = 0.0;
  private double feedRightDistance = 0.0;

  private double usedFeedDistance = 0.0;

  public RobotManager(
      ShooterHood shooterHood,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      DyeRotor dyeRotor,
      Turret turret,
      Intake intake,
      Deploy deploy,
      Vision vision,
      Lights lights) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.shooterHood = shooterHood;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.dyeRotor = dyeRotor;
    this.turret = turret;
    this.intake = intake;
    this.deploy = deploy;
    this.vision = vision;
    this.lights = lights;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case PREPARE_SCORE -> {
        if (shooter.atGoal()
            && FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()
            && vision.seeingTagDebounced()) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case PREPARE_FORCE_SCORE -> {
        if (shooter.atGoal() && dyeRotor.atGoal() && turret.atGoal() && shooterHood.atGoal()) {
          yield RobotState.FORCE_SCORE;
        }
        yield currentState;
      }
      case PREPARE_FEED_LEFT ->
          shooter.atGoal()
                  && (!FieldUtil.isRobotInNoFeedZone(robotPose)
                      && dyeRotor.atGoal()
                      && turret.atGoal()
                      && shooterHood.atGoal())
              ? RobotState.FEED_LEFT
              : currentState;
      case PREPARE_FEED_RIGHT ->
          shooter.atGoal()
                  && (!FieldUtil.isRobotInNoFeedZone(robotPose)
                      && dyeRotor.atGoal()
                      && turret.atGoal()
                      && shooterHood.atGoal())
              ? RobotState.FEED_RIGHT
              : currentState;
      case SCORE -> {
        // If we are not in the alliance zone while vision is online, stop tracking the hub.
        // Otherwise, if vision is dead and we cannot reliable track whether we are in the alliance
        // zone, we still want to be able to score
        if (!FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())
            && vision.isAnyCameraOnlineForTags()) {
          yield RobotState.IDLE;
        }
        if (shooter.atGoal() == false) {
          yield RobotState.PREPARE_SCORE;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
      // Set hood behavior separately while idling
        dyeRotor.idleRequest();
        turret.idleRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_EMPTY);
      }
      case PREPARE_FORCE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case FORCE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case PREPARE_FEED_LEFT -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedLeftDistance);
        shooterHood.feedRequest(feedLeftDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case PREPARE_FEED_RIGHT -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedRightDistance);
        shooterHood.feedRequest(feedRightDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case FEED_LEFT -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedLeftDistance);
        shooterHood.feedRequest(feedLeftDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case FEED_RIGHT -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedRightDistance);
        shooterHood.feedRequest(feedRightDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE -> shooterHoodSmartIdleRequest();
      case PREPARE_FEED_LEFT, FEED_LEFT -> {
        turret.setFeedAimAngle(feedLeftGoalAngle);
        usedFeedDistance = feedLeftDistance;
      }
      case PREPARE_FEED_RIGHT, FEED_RIGHT -> {
        turret.setFeedAimAngle(feedRightGoalAngle);
        usedFeedDistance = feedRightDistance;
      }
      case PREPARE_SCORE, SCORE -> turret.setHubAimAngle(hubGoalAngle);
      default -> {}
    }
  }

  public void idleRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.IDLE);
    }
  }

  public void forceShootRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_FORCE_SCORE);
    }
  }

  public void shootHubWaitRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_SCORE);
    }
  }

  public void feedLeftWaitRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_FEED_LEFT);
    }
  }

  public void feedRightWaitRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_FEED_RIGHT);
    }
  }

  public void toggleHubRequest() {
    if (!getState().isClimbingOrRehoming()) {
      switch (getState()) {
        case PREPARE_SCORE, SCORE -> setStateFromRequest(RobotState.IDLE);
        default -> setStateFromRequest(RobotState.PREPARE_SCORE);
      }
    }
  }

  public void setFeedGoalLeftRequest() {
    feedLocation = FeedLocation.LEFT;
  }

  public void setFeedGoalRightRequest() {
    feedLocation = FeedLocation.RIGHT;
  }

  public void intakeRequest() {
    intake.intakeRequest();
    deploy.intakeRequest();
  }

  private void shooterHoodSmartIdleRequest() {
    // TODO: Remove logs later
    DogLog.timestamp("RobotManager/HoodSmartIdle");

    // -First, if cameras are offline or we are near a trench, always be idle
    // -Otherwise if we are in our alliance zone, point towards hub
    // -And if we are not in alliance zone, point towards feed pose
    if (!vision.isAnyCameraOnlineForTags() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/ShooterHoodSmartIdleRequest", "NearTrench");
    } else if (FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
      shooterHood.scoreRequest(hubDistance);
      DogLog.log("RobotManager/ShooterHoodSmartIdleRequest", "InAllianceZone");
    } else {
      shooterHood.feedRequest(usedFeedDistance);
      DogLog.log("RobotManager/ShooterHoodSmartIdleRequest", "NotInAlliance");
    }
  }
  public void cancelIntakeRequest() {
    intake.idleRequest();
    deploy.stowRequest();
    swerve.normalDriveRequest();
  }

  public void toggleFeedRequest() {
    if (!getState().isClimbingOrRehoming()) {
      switch (getState()) {
        case PREPARE_FEED_LEFT, PREPARE_FEED_RIGHT, FEED_LEFT, FEED_RIGHT ->
            setStateFromRequest(RobotState.IDLE);
        default -> {
          if (feedLocation == FeedLocation.LEFT) {
            setStateFromRequest(RobotState.PREPARE_FEED_LEFT);
          } else {
            setStateFromRequest(RobotState.PREPARE_FEED_RIGHT);
          }
        }
      }
    }
  }

  public void climbSequenceForward() {
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
    switch (getState()) {
      default -> setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTO);
      case CLIMB_1_LINEUP_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTO);
      case CLIMB_2_RAISING_L1_AUTO -> setStateFromRequest(RobotState.CLIMB_3_HANGING_L1_AUTO);
      case CLIMB_3_HANGING_L1_AUTO -> {}
    }
  }

  public void climbSequenceBackward() {
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

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    nearTrench =
        FieldUtil.inTrench(robotPose.getTranslation())
            || SwerveAssist.ableToTrenchAssist(robotPose, swerve.getFieldRelativeSpeeds());

    hubGoalAngle = 0.0;
    hubDistance = 0.0;
    feedLeftGoalAngle = 0.0;
    feedLeftDistance = 0.0;
    feedRightGoalAngle = 0.0;
    feedRightDistance = 0.0;
    var yDistanceToLeft = Math.abs(robotPose.getY() - FieldUtil.FEED_LEFT_POSE.getY());
    var yDistanceToRight = Math.abs(robotPose.getY() - FieldUtil.FEED_RIGHT_POSE.getY());
    var closestFeedLocation = Math.min(yDistanceToLeft, yDistanceToRight);
    if(closestFeedLocation == yDistanceToLeft) {
      feedLocation = FeedLocation.LEFT;
    } else {
      feedLocation = FeedLocation.RIGHT;
    }
  }
}
