package frc.robot.robot_manager;

import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.deploy.Deploy;
import frc.robot.dye_rotor.DyeRotor;
import frc.robot.imu.Imu;
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

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  private final ShooterHood shooterHood;
  private final Shooter shooter;
  private final DyeRotor dyeRotor;
  private final Turret turret;
  private final Intake intake;

  private final Lights lights;

  private Pose2d robotPose = Pose2d.kZero;
  private boolean nearTrench = false;

  private Translation2d hubGoalPose = Translation2d.kZero;
  private double hubGoalAngle = 0.0;
  private double hubDistance = 0.0;

  private double feedLeftGoalAngle = 0.0;
  private double feedLeftDistance = 0.0;

  private double feedRightGoalAngle = 0.0;
  private double feedRightDistance = 0.0;

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

    this.lights = lights;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      case PREPARE_SCORE -> {
        if (shooter.atGoal()
            && FieldUtil.isRobotInAllianceZone(robotPose)
            && dyeRotor.atGoal()
            && turret.atGoal()
            && shooterHood.atGoal()) {
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
        if (!FieldUtil.isRobotInAllianceZone(robotPose)) {
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
        swerve.normalDriveRequest();
        shooterHood.idleRequest();
        shooter.idleRequest();
        intake.idleRequest();
        dyeRotor.idleRequest();
        turret.idleRequest();
        lights.setState(LightsState.IDLE_EMPTY);
      }
      case PREPARE_FORCE_SCORE -> {
        shooter.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        shooterHood.scoreRequest(hubDistance);
        turret.hubAimRequest();
        swerve.normalDriveRequest();
      }
      case FORCE_SCORE -> {
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        dyeRotor.shootRequest();
        intake.shootingRequest();
        turret.hubAimRequest();
        swerve.normalDriveRequest();
      }
      case WAIT_FEED_LEFT, PREPARE_FEED_LEFT -> {
        shooter.feedRequest(feedLeftDistance);
        shooterHood.feedRequest(feedLeftDistance);
        turret.feedAimRequest();
        dyeRotor.shootRequest();
        swerve.normalDriveRequest();
      }
      case WAIT_FEED_RIGHT, PREPARE_FEED_RIGHT -> {
        shooter.feedRequest(feedRightDistance);
        shooterHood.feedRequest(feedRightDistance);
        turret.feedAimRequest();
        dyeRotor.shootRequest();
        swerve.normalDriveRequest();
      }
      case FEED_LEFT -> {
        shooter.feedRequest(feedLeftDistance);
        shooterHood.feedRequest(feedLeftDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case FEED_RIGHT -> {
        shooter.feedRequest(feedRightDistance);
        shooterHood.feedRequest(feedRightDistance);
        dyeRotor.shootRequest();
        turret.feedAimRequest();
        intake.shootingRequest();
        swerve.normalDriveRequest();
      }
      case WAIT_SCORE, PREPARE_SCORE -> {
        shooter.scoreRequest(hubDistance);
        shooterHood.scoreRequest(hubDistance);
        // Intake is controlled separately
        dyeRotor.shootRequest();
        turret.hubAimRequest();
        swerve.normalDriveRequest();
      }
      case SCORE -> {
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
      case IDLE -> {
        if (nearTrench) {
          shooterHood.idleRequest();
        } else {
          shooterHood.scoreRequest(0);
        }
      }
      case WAIT_FEED_LEFT, PREPARE_FEED_LEFT, FEED_LEFT ->
          turret.setFeedAimAngle(feedLeftGoalAngle);
      case WAIT_FEED_RIGHT, PREPARE_FEED_RIGHT, FEED_RIGHT ->
          turret.setFeedAimAngle(feedRightGoalAngle);
      case WAIT_SCORE, PREPARE_SCORE, SCORE -> turret.setHubAimAngle(hubGoalAngle);
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    nearTrench = FieldUtil.getCurrentTrenchAssistZone(robotPose.getTranslation()).isPresent();
  }
}
