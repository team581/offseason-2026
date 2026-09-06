package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerBindings;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.autos.BumpCrossingFollower;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.funneler.Funneler;
import frc.robot.generated.BuildConstants;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.imu.Imu;
import frc.robot.intake.Intake;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.vision.Vision;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final Imu imu = new Imu(hardware.drivetrain);
  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new BumpCrossingFollower(
              new PidPathFollower(
                  new PIDController(3.5, 0, 0),
                  new PIDController(
                      Swerve.ORIGINAL_HEADING_PID.getP(),
                      Swerve.ORIGINAL_HEADING_PID.getI(),
                      Swerve.ORIGINAL_HEADING_PID.getD())),
              imu.bumpCrossingTracker));

  private final HealthManager health =
      new HealthManager(
          hardware.shooterLimeLight,
          hardware.leftLimeLight,
          hardware.rightLimeLight,
          hardware.groundLimeLight);
  private final Vision vision =
      new Vision(
          imu,
          hardware.shooterLimeLight,
          hardware.leftLimeLight,
          hardware.rightLimeLight,
          hardware.groundLimeLight);
  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);
  private final Shooter shooter =
      new Shooter(
          hardware.shooterTopLeftMotor,
          hardware.shooterTopRightMotor,
          hardware.shooterBottomLeftMotor,
          hardware.shooterBottomRightMotor);

  private final Intake intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
  private final Deploy deploy = new Deploy(hardware.deployDifferentialMechanism);
  private final Feeder feeder = new Feeder(hardware.feederTopMotor, hardware.feederBottomMotor);
  private final Conveyor conveyor =
      new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);
  private final Funneler funneler = new Funneler(hardware.funnelerMotor);

  private final Turret turret =
      new Turret(
          hardware.turretMotor,
          hardware.turretEncoder,
          vision,
          swerve::getDriverIntentTurretFeedForwardCorrection);

  private final HubActivity hubActivity = new HubActivity();
  private final HopperManager hopperManager =
      new HopperManager(
          deploy, intake, conveyor, feeder, hardware.hopperCANRange, hardware.towerSensor);
  private final PowerManager powerManager =
      new PowerManager(
          shooter, intake, deploy, shooterHood, feeder, conveyor, funneler, swerve, turret);
  private final RobotManager robotManager =
      new RobotManager(
          hopperManager,
          shooterHood,
          localization,
          swerve,
          shooter,
          turret,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          trailblazer,
          hardware,
          powerManager);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    finalizeInit();
  }

  @Override
  protected void configureBindings() {
    var driver =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.driverController);
    var operator =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.operatorController);

    driver.back().onPress(localization::zeroGyro);

    driver
        .leftTrigger()
        .onPress(robotManager::intakeAutoRequest)
        .onRelease(robotManager::cancelIntakeRequest);

    driver
        .rightTrigger()
        .onPress(robotManager::prepareScoreOrFeedRequest)
        .onRelease(robotManager::idleRequest);

    driver.rightBumper().onPress(robotManager::stowDeployRequest);

    driver.leftBumper().onPress(robotManager::unjamRequest).onRelease(robotManager::idleRequest);

    operator.leftTrigger().onPress(robotManager::stowDeployRequest);
    operator.rightTrigger().onPress(robotManager::prepareScoreRequest);

    operator.leftBumper().onPress(powerManager::prioritizeIntakeRequest);
    operator
        .rightBumper()
        .onPress(() -> robotManager.setTrenchOverrideRequest(true))
        .onRelease(() -> robotManager.setTrenchOverrideRequest(false));

    operator.start().onPress(() -> hopperManager.deploy.homingRequest());

    operator.back().onPress(robotManager::homeShooterHoodRequest);
  }
}
