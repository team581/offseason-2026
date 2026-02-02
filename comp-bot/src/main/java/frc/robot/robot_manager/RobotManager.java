package frc.robot.robot_manager;

import com.team581.swerve.SwerveAssist;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.health.HealthManager;
import frc.robot.intake.Intake;
import frc.robot.lights.Lights;
import frc.robot.lights.LightsState;
import frc.robot.localization.Localization;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.shooter_hood.ShooterHoodState;
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
      // No auto transitions for these states
      case IDLE,
          UNJAM,
          MANUAL_CLIMB_1_LINEUP_L1,
          MANUAL_CLIMB_2_RAISING_L1,
          MANUAL_CLIMB_3_HANGING_L1,
          MANUAL_CLIMB_4_RAISING_L2,
          MANUAL_CLIMB_5_HANGING_L2,
          MANUAL_CLIMB_6_RAISING_L3,
          MANUAL_CLIMB_7_HANGING_L3,
          AUTOMATIC_CLIMB_7_HANGING_L3 ->
          currentState;
      case REHOME_DEPLOY -> {
        if (deploy.getState() == DeployState.STOWED) {
          yield RobotState.IDLE;
        }
        yield currentState;
      }
      case REHOME_SHOOTER_HOOD -> {
        if (shooterHood.getState() == ShooterHoodState.IDLE) {
          yield RobotState.IDLE;
        }
        yield currentState;
      }
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
                  // If localization is healthy, you can feed if we're not in a no-feed zone
                  // If localization is dead, you can always shoot
                  && (health.isLocalizationHealthy()
                      ? !FieldUtil.isRobotInNoFeedZone(robotPose)
                      : true)
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

        if (shooter.atGoal()
            && localization.isTrustworthy()
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()) {
          yield currentState;
        }

        yield RobotState.PREPARE_SCORE;
      }
      case FEED ->
          shooter.atGoal()
                  && (health.isLocalizationHealthy()
                      ? !FieldUtil.isRobotInNoFeedZone(robotPose)
                      : true)
                  && dyeRotor.atGoal()
                  && turret.atGoal()
                  && shooterHood.atGoal()
              ? currentState
              : RobotState.PREPARE_FEED;

      // TODO: When climber is done, fill out the automatic climb logic
      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_2
        yield currentState;
      }
      case AUTOMATIC_CLIMB_2_RAISING_L1 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_3
        yield currentState;
      }
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_4
        yield currentState;
      }
      case AUTOMATIC_CLIMB_4_RAISING_L2 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_5
        yield currentState;
      }
      case AUTOMATIC_CLIMB_5_HANGING_L2 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_6
        yield currentState;
      }
      case AUTOMATIC_CLIMB_6_RAISING_L3 -> {
        // If climber is at goal, set state to AUTOMATIC_CLIMB_7
        yield currentState;
      }
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> {
        // If climber is at goal, set state to CLIMB_2_AUTONOMOUS
        yield currentState;
      }
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> {
        // If climber is at goal, set state to CLIMB_3_AUTONOMOUS
        yield currentState;
      }
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> {
        // If climber is at goal && we have transitioned into teleop, set state to
        // CLIMB_4_RELEASE_AUTONOMOUS
        yield currentState;
      }
      case CLIMB_4_RELEASE_L1_AUTONOMOUS -> {
        // If climber is at goal(we are completely released), set state to IDLE
        yield currentState;
      }
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case IDLE -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        // Set hood behavior separately whiile idling/unjamming/rehoming
        // Set turret behavior separately while
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
      }
      case PREPARE_FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedRequest(feedingParameters.turretAngle());
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
      }
      case FEED -> {
        vision.setState(VisionState.TAGS);
        shooter.feedRequest(feedingParameters.distance());
        shooterHood.feedRequest(feedingParameters.distance());
        dyeRotor.shootRequest();
        turret.feedRequest(feedingParameters.turretAngle());
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
      }
      case PREPARE_SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        // Intake is controlled separately
        swerve.normalDriveRequest();
        lights.setState(LightsState.WAITING_TO_SHOOT);
      }
      case SCORE -> {
        vision.setState(VisionState.HUB_TAGS);
        shooter.scoreRequest(scoringParameters.distance());
        shooterHood.scoreRequest(scoringParameters.distance());
        dyeRotor.shootRequest();
        turret.scoreRequest(scoringParameters.turretAngle());
        intake.shootingRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.SHOOTING);
      }
      case UNJAM -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.unjamRequest();
        // Set turret behavior separately
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
      }
      case REHOME_DEPLOY -> {
        deploy.homingRequest();

        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.idleRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separately
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
      }
      case REHOME_SHOOTER_HOOD -> {
        vision.setState(VisionState.TAGS);
        shooter.idleRequest();
        shooterHood.homingRequest();
        dyeRotor.idleRequest();
        // Set turret behavior separately
        intake.idleRequest();
        swerve.normalDriveRequest();
        lights.setState(LightsState.IDLE_INTAKE_NOT_FULL);
      }
      case MANUAL_CLIMB_1_LINEUP_L1 -> {
        lights.setState(LightsState.CLIMB_1);
      }
      case MANUAL_CLIMB_2_RAISING_L1 -> {
        lights.setState(LightsState.CLIMB_2);
      }
      case MANUAL_CLIMB_3_HANGING_L1 -> {
        lights.setState(LightsState.CLIMB_3);
      }
      case MANUAL_CLIMB_4_RAISING_L2 -> {
        lights.setState(LightsState.CLIMB_4);
      }
      case MANUAL_CLIMB_5_HANGING_L2 -> {
        lights.setState(LightsState.CLIMB_5);
      }
      case MANUAL_CLIMB_6_RAISING_L3 -> {
        lights.setState(LightsState.CLIMB_6);
      }
      case MANUAL_CLIMB_7_HANGING_L3 -> {
        lights.setState(LightsState.CLIMB_7);
      }
      case AUTOMATIC_CLIMB_1_LINEUP_L1 -> {
        lights.setState(LightsState.CLIMB_1);
      }
      case AUTOMATIC_CLIMB_2_RAISING_L1 -> {
        lights.setState(LightsState.CLIMB_2);
      }
      case AUTOMATIC_CLIMB_3_HANGING_L1 -> {
        lights.setState(LightsState.CLIMB_3);
      }
      case AUTOMATIC_CLIMB_4_RAISING_L2 -> {
        lights.setState(LightsState.CLIMB_4);
      }
      case AUTOMATIC_CLIMB_5_HANGING_L2 -> {
        lights.setState(LightsState.CLIMB_5);
      }
      case AUTOMATIC_CLIMB_6_RAISING_L3 -> {
        lights.setState(LightsState.CLIMB_6);
      }
      case AUTOMATIC_CLIMB_7_HANGING_L3 -> {
        lights.setState(LightsState.CLIMB_7);
      }
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> {
        lights.setState(LightsState.CLIMB_1);
      }
      case CLIMB_2_RAISING_L1_AUTONOMOUS -> {
        lights.setState(LightsState.CLIMB_2);
      }
      case CLIMB_3_HANGING_L1_AUTONOMOUS -> {
        lights.setState(LightsState.CLIMB_3);
      }
      case CLIMB_4_RELEASE_L1_AUTONOMOUS -> {
        lights.setState(LightsState.CLIMB_4);
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    switch (state) {
      case IDLE, REHOME_DEPLOY, REHOME_SHOOTER_HOOD, UNJAM -> {
        shooterHoodSmartIdleRequest();
        turretSmartIdleRequest();
      }
      case PREPARE_SCORE -> {
        turret.scoreRequest(scoringParameters.turretAngle());
      }
      case SCORE -> {
        turret.scoreRequest(scoringParameters.turretAngle());
      }
      case PREPARE_FEED -> {
        turret.feedRequest(feedingParameters.turretAngle());
      }
      case FEED -> {
        turret.feedRequest(feedingParameters.turretAngle());
      }
      default -> {}
    }
    DogLog.log("RobotManager/FeedLocation", feedLocation);
    DogLog.log("RobotManager/FeedParameters", feedingParameters);
    DogLog.log("RobotManager/ScoringParameters", scoringParameters);

    MechanismVisualizer.log(
        robotPose, turret.getAngle(), shooterHood.getAngle(), deploy.getPosition(), 0);
  }

  private void shooterHoodSmartIdleRequest() {
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

  private void turretSmartIdleRequest() {
    // -First, if cameras are offline or we are near a trench, always be idle
    // -Otherwise if we are in our alliance zone, point towards hub
    // -And if we are not in alliance zone, point towards feed pose
    if (health.isLocalizationHealthy() && nearTrench) {
      turret.idleScoreRequest(scoringParameters.turretAngle());
      DogLog.log("RobotManager/TurretSmartIdleRequest", "NearTrench");
    } else if (FieldUtil.isRobotInAllianceZone(robotPose.getTranslation())) {
      turret.idleScoreRequest(scoringParameters.turretAngle());
      DogLog.log("RobotManager/TurretSmartIdleRequest", "InAllianceZone");
    } else {
      turret.idleFeedReuqest(feedingParameters.turretAngle());
      DogLog.log("RobotManager/TurretSmartIdleRequest", "NotInAlliance");
    }
  }

  public void idleRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.IDLE);
    }
  }

  public void prepareScoreRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.PREPARE_SCORE);
    }
  }

  public void prepareFeedRequest() {
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

  public void cancelIntakeRequest() {
    intake.idleRequest();
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

  public void unjamRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void rehomeDeployRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.REHOME_DEPLOY);
    }
  }

  public void rehomeShooterHoodRequest() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.REHOME_SHOOTER_HOOD);
    }
  }

  public void startAutoClimbSequence() {
    setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTONOMOUS);
  }

  public void startTeleopAutoClimbSequence() {
    if (!getState().isClimbingOrRehoming()) {
      setStateFromRequest(RobotState.AUTOMATIC_CLIMB_1_LINEUP_L1);
    }
  }

  public void manualClimbSequenceForward() {
    switch (getState()) {
      default -> setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_LINEUP_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_RAISING_L1);
      case MANUAL_CLIMB_2_RAISING_L1, AUTOMATIC_CLIMB_2_RAISING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_HANGING_L1);
      case MANUAL_CLIMB_3_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_RAISING_L2);

      case MANUAL_CLIMB_4_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_HANGING_L2);
      case MANUAL_CLIMB_5_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_6_RAISING_L3);

      case MANUAL_CLIMB_6_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_7_HANGING_L3);
      case MANUAL_CLIMB_7_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 -> {}
    }
  }

  public void manualClimbSequenceBackward() {
    switch (getState()) {
      default -> {}
      case CLIMB_1_LINEUP_L1_AUTONOMOUS -> setStateFromRequest(RobotState.IDLE);
      case CLIMB_2_RAISING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_1_LINEUP_L1_AUTONOMOUS);
      case CLIMB_3_HANGING_L1_AUTONOMOUS ->
          setStateFromRequest(RobotState.CLIMB_2_RAISING_L1_AUTONOMOUS);

      case MANUAL_CLIMB_1_LINEUP_L1, AUTOMATIC_CLIMB_1_LINEUP_L1 ->
          setStateFromRequest(RobotState.IDLE);
      case MANUAL_CLIMB_2_RAISING_L1, AUTOMATIC_CLIMB_2_RAISING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_1_LINEUP_L1);
      case MANUAL_CLIMB_3_HANGING_L1, AUTOMATIC_CLIMB_3_HANGING_L1 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_2_RAISING_L1);

      case MANUAL_CLIMB_4_RAISING_L2, AUTOMATIC_CLIMB_4_RAISING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_3_HANGING_L1);
      case MANUAL_CLIMB_5_HANGING_L2, AUTOMATIC_CLIMB_5_HANGING_L2 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_4_RAISING_L2);

      case MANUAL_CLIMB_6_RAISING_L3, AUTOMATIC_CLIMB_6_RAISING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_5_HANGING_L2);
      case MANUAL_CLIMB_7_HANGING_L3, AUTOMATIC_CLIMB_7_HANGING_L3 ->
          setStateFromRequest(RobotState.MANUAL_CLIMB_6_RAISING_L3);
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
