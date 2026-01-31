package frc.robot.robot_manager;

import com.team581.math.SwerveAssist;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.health.HealthManager;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.lights.LightsState;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.AimParameterUtil.AimingParameters;
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
  private final HealthManager health;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private AimingParameters scoringParameters = new AimingParameters(0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0);

  private FeedLocation feedLocation = FeedLocation.CLOSEST;

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
      Lights lights,
      HealthManager health) {
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
    this.health = health;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case PREPARE_SCORE -> {
        if (shooter.atGoal()
            && localization.isTrustworthy()
            && FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()) {
          yield RobotState.SCORE;
        }
        yield currentState;
      }
      case PREPARE_FEED ->
          shooter.atGoal()
                  && (!health.isLocalizationHealthy() || FieldUtil.isRobotInNoFeedZone(robotPose))
                  && dyeRotor.atGoal()
                  && turret.atGoal()
                  && shooterHood.atGoal()
              ? RobotState.FEED
              : currentState;
      case SCORE -> {
        // If we are not in the alliance zone while vision is online, stop tracking the hub.
        // Otherwise, if vision is dead and we cannot reliable track whether we are in the alliance
        // zone, we still want to be able to score
        if (health.isLocalizationHealthy()
            && !FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
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
        turret.feedAimRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_EMPTY);
      }
      case PREPARE_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        // Intake is controlled separately
        swerve.normalDriveRequest();
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case CLIMB_1_LINEUP_L1 -> {}
      case CLIMB_2_RAISING_L1 -> {}
      case CLIMB_3_HANGING_L1 -> {}
      case CLIMB_4_RAISING_L2 -> {}
      case CLIMB_5_HANGING_L2 -> {}
      case CLIMB_6_RAISING_L3 -> {}
      case CLIMB_7_HANGING_L3 -> {}
      case CLIMB_1_LINEUP_L1_AUTO -> {}
      case CLIMB_2_RAISING_L1_AUTO -> {}
      case CLIMB_3_HANGING_L1_AUTO -> {}
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE -> shooterHoodSmartIdleRequest();
      case PREPARE_FEED, FEED -> {
        turret.setFeedAimAngle(feedingParameters.angle());
      }
      case PREPARE_SCORE, SCORE -> turret.setHubAimAngle(scoringParameters.angle());
      default -> {}
    }
    DogLog.log("RobotManager/FeedLocation", feedLocation);
    DogLog.log("RobotManager/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/ScoringParameters", scoringParameters);

    MechanismVisualizer.log(
        robotPose, turret.getAngle(), shooterHood.getAngle(), deploy.getPosition(), 0);
  }

  public void idleRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.IDLE);
    }
  }

  public void shootHubWaitRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_SCORE);
    }
  }

  public void feedPrepareRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_FEED);
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
    if (!health.isLocalizationHealthy() || nearTrench) {
      shooterHood.idleRequest();
      DogLog.log("RobotManager/ShooterHoodSmartIdleRequest", "NearTrench");
    } else if (FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
      shooterHood.scoreRequest(scoringParameters.distance());
      DogLog.log("RobotManager/ShooterHoodSmartIdleRequest", "InAllianceZone");
    } else {
      shooterHood.feedRequest(feedingParameters.distance());
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
        case PREPARE_FEED, FEED -> setStateFromRequest(RobotState.IDLE);
        default -> {
          setStateFromRequest(RobotState.PREPARE_FEED);
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
    vision.setEstimatedPoseAngle(robotPose.getRotation().getDegrees());

    nearTrench =
        FieldUtil.inTrench(robotPose.getTranslation())
            || SwerveAssist.ableToTrenchAssist(robotPose, swerve.getFieldRelativeSpeeds());

    scoringParameters =
        AimParameterUtil.getScoringParameters(
            robotPose, swerve.getFieldRelativeSpeeds(), shooter.getCurrentTimeOfFlight());
    feedingParameters =
        AimParameterUtil.getFeedingParameters(
            feedLocation,
            robotPose,
            swerve.getFieldRelativeSpeeds(),
            shooter.getCurrentTimeOfFlight());
  }
}
