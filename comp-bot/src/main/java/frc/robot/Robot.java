package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.Base581Robot;
import com.team581.GlobalConfig;
import com.team581.controller.ControllerBindings;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.profiling.DiagnosticCadence;
import com.team581.util.profiling.LoopTiming;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.autos.Autos;
import frc.robot.autos.BumpCrossingFollower;
import frc.robot.cluster_map.ClusterMap;
import frc.robot.config.FeatureFlags;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
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
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

public class Robot extends Base581Robot {
  private static DogLogOptions compDogLogOptions() {
    return new DogLogOptions()
        .withCaptureDs(true)
        .withNtPublish(GlobalConfig.IS_DEVELOPMENT)
        .withNtTunables(GlobalConfig.IS_DEVELOPMENT)
        .withLogEntryQueueCapacity(4096)
        .withUseLogThread(true);
  }

  private final Hardware hardware = new Hardware();
  private final Limelight shooterLimelight =
      new Limelight("shooter", LimelightState.TAGS, CameraConfigs.SHOOTER);
  private final Limelight leftLimelight =
      new Limelight("left", LimelightState.TAGS, CameraConfigs.LEFT);
  private final Limelight rightLimelight =
      new Limelight("right", LimelightState.TAGS, CameraConfigs.RIGHT);
  private final Limelight groundLimelight =
      new Limelight("ground", LimelightState.CLUSTER_MAP, CameraConfigs.GROUND);

  private final HealthManager health =
      new HealthManager(shooterLimelight, leftLimelight, rightLimelight, groundLimelight);

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

  private final Swerve swerve =
      new Swerve(hardware.drivetrain, health, hardware.driverController, trailblazer);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);
  private final Shooter shooter =
      new Shooter(
          hardware.shooterTopLeftMotor,
          hardware.shooterTopRightMotor,
          hardware.shooterBottomLeftMotor,
          hardware.shooterBottomRightMotor);
  private final Intake intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
  private final Deploy deploy = new Deploy(hardware.deployDifferentialMechanism);
  private final Vision vision =
      new Vision(imu, shooterLimelight, leftLimelight, rightLimelight, groundLimelight);
  private final Localization localization =
      new Localization(swerve, hardware.drivetrain, vision, imu);
  private final Feeder feeder = new Feeder(hardware.feederTopMotor, hardware.feederBottomMotor);

  private final Conveyor conveyor =
      new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);
  private final ClusterMap clusterMap = new ClusterMap(localization, swerve, groundLimelight);

  private final HubActivity hubActivity = new HubActivity();
  private final PowerManager powerManager =
      new PowerManager(shooter, intake, deploy, shooterHood, feeder, conveyor, swerve);

  private final HopperManager hopperManager =
      new HopperManager(
          deploy, intake, conveyor, feeder, hardware.hopperCANRange, hardware.towerSensor);

  private final RobotManager robotManager =
      new RobotManager(
          hopperManager,
          shooterHood,
          localization,
          swerve,
          shooter,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          trailblazer,
          clusterMap,
          hardware,
          powerManager);

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    super(compDogLogOptions());

    DiagnosticCadence.setEnabled(true);
    LoopTiming.setEnabled(true);

    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    // Reuse the SwerveDriveState from Swerve for IMU
    imu.setDriveStateSupplier(swerve::getDriveState);

    finalizeInit();

    if (GlobalConfig.IS_DEVELOPMENT) {
      FieldUtil.debugLogFieldZones();
    }

    if (RobotBase.isSimulation()) {
      try {
        var docsDir = Path.of(System.getProperty("user.dir")).resolve("../docs");
        Files.writeString(
            docsDir.resolve("feeding_obstructions.svg"), FieldUtil.FEEDING_OBSTRUCTIONS.toSvg());
      } catch (IOException e) {
        throw new RuntimeException("Failed to write field obstacles SVG", e);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    DiagnosticCadence.beginLoop();
    LoopTiming.recordClockOverhead();
    long loopStart = LoopTiming.start();

    super.robotPeriodic();

    if (FeatureFlags.CLAMPED_AUTO_POINTS.getAsBoolean() && !FmsUtil.isRedAlliance()) {
      DogLog.logFault("Clamped auto points are enabled but current alliance is blue");
    } else {
      DogLog.clearFault("Clamped auto points are enabled but current alliance is blue");
    }

    LoopTiming.end("Scheduler/RobotPeriodicExecution", loopStart);
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
        .onPress(() -> hopperManager.setDriverWantsIntake(true))
        .onRelease(() -> hopperManager.setDriverWantsIntake(false));

    driver
        .rightTrigger()
        .onPress(robotManager::prepareScoreOrFeedRequest)
        .onRelease(robotManager::idleRequest);

    driver.rightBumper().onPress(robotManager::idleRequest);

    driver
        .leftBumper()
        .onPress(() -> hopperManager.setDriverWantsEject(true))
        .onRelease(() -> hopperManager.setDriverWantsEject(false));

    operator.start().onPress(() -> hopperManager.deploy.homingRequest());

    operator.back().onPress(robotManager::homeShooterHoodRequest);

    operator.x().onPress(robotManager::unjamRequest).onRelease(robotManager::idleRequest);

    operator
        .y()
        .onPress(
            () -> {
              powerManager.turboRequest();
              shooter.setTurboMode(true);
            })
        .onRelease(
            () -> {
              powerManager.idleRequest();
              shooter.setTurboMode(false);
            });

    // operator.b().onPress(robotManager::prepareFeedRequest).onRelease(robotManager::idleRequest);

    operator.b().onPress(deploy::fixDifferentialDesyncRequest).onRelease(deploy::intakeRequest);

    operator
        .rightTrigger()
        .onPress(robotManager::warmupScoreRequest)
        .onRelease(robotManager::cancelWarmupRequest);

    operator
        .leftTrigger()
        .onPress(robotManager::stowDeployRequest)
        .onRelease(robotManager::cancelStowDeployRequest);

    operator
        .leftBumper()
        .onPress(() -> robotManager.powerManager.prioritizeIntakeRequest())
        .onRelease(() -> robotManager.powerManager.idleRequest());
    operator
        .rightBumper()
        .onPress(() -> robotManager.setTrenchOverrideRequest(true))
        .onRelease(() -> robotManager.setTrenchOverrideRequest(false));
  }

  String benchmarkBehaviorSnapshot() {
    return robotManager.getState()
        + "|"
        + hopperManager.getState()
        + "|"
        + swerve.getState()
        + "|"
        + shooter.getState()
        + "|"
        + shooterHood.getState()
        + "|"
        + intake.getState()
        + "|"
        + conveyor.getState()
        + "|"
        + feeder.getState()
        + "|"
        + deploy.getState()
        + "|"
        + swerve.getRequestedSpeeds()
        + "|"
        + shooterHood.getAngle()
        + "|"
        + deploy.getPosition()
        + "|"
        + localization.getPose()
        + "|"
        + clusterMap.getBestClusterLane()
        + "|"
        + clusterMap.getBestClusterPose();
  }

  /** Removes the native CAN-dependent odometry worker from deterministic desktop benchmarks. */
  void benchmarkInitializeSimulation() {
    hardware.drivetrain.getOdometryThread().stop();
  }

  /** Seeds deterministic desktop-simulation inputs before a benchmark loop. */
  void benchmarkSimulationStep(double elapsedSeconds) {
    hardware.drivetrain.updateSimState(0.020, 12.0);

    double rotorVelocity = 5.0 * Math.sin(elapsedSeconds * 0.5);
    TalonFX[] motors = {
      hardware.deployDifferentialMechanism.getLeader(),
      hardware.deployDifferentialMechanism.getFollower(),
      hardware.intakeLeftMotor,
      hardware.intakeRightMotor,
      hardware.conveyorTopMotor,
      hardware.conveyorBottomMotor,
      hardware.feederTopMotor,
      hardware.feederBottomMotor,
      hardware.shooterHoodMotor,
      hardware.shooterBottomLeftMotor,
      hardware.shooterBottomRightMotor,
      hardware.shooterTopLeftMotor,
      hardware.shooterTopRightMotor
    };
    for (var motor : motors) {
      motor.getSimState().setSupplyVoltage(12.0);
      motor.getSimState().setRotorVelocity(rotorVelocity);
    }

    hardware.hopperCANRange.getSimState().setSupplyVoltage(12.0);
    hardware.hopperCANRange.getSimState().setDistance(0.20 + 0.04 * Math.sin(elapsedSeconds));
  }
}
