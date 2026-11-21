package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.autos.AutoChooser;
import com.team581.math.MathHelpers;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FmsUtil;
import com.team581.util.profiling.LoopTiming;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompDriveConfig;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.Swerve.TrajectoryOwner;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.jspecify.annotations.Nullable;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private record SelectedAuto(
      AutoSelection selection, Alliance alliance, TrailblazerAuto<?> auto) {}

  private static final String WARMUP_RUNNING_FAULT = "Trailblazer warmup still running";

  private static final String PATH_CALCULATION_RUNNING_FAULT =
      "Trailblazer path calculation still running";

  private static Executor createPathExecutor() {
    return Executors.newSingleThreadExecutor(task -> {
      var worker = new Thread(task, "trailblazer-path-planner");
      worker.setDaemon(true);
      return worker;
    });
  }

  private static Alliance currentAlliance() {
    return FmsUtil.isRedAlliance() ? Alliance.Red : Alliance.Blue;
  }

  private static boolean isStationaryForLaunch(ChassisSpeeds speeds) {
    return MathHelpers.getLinearVelocity(speeds) <= 0.10
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Math.toRadians(10.0));
  }

  // Pre-warms Trailblazer classloading and JIT paths off the main loop.
  private static void warmUpTrailblazer() {
    double dt = CompDriveConfig.FOLLOWER_PERIOD_SECONDS;
    double[] syntheticClock = {1.0};
    var warmupPlanner =
        new Trailblazer(
            () -> syntheticClock[0],
            CompDriveConfig.DRIVE_SPEC,
            CompDriveConfig.CALIBRATION,
            dt,
            Optional.of(CompAutoField.CLEARANCE));
    Pose2d startPose = CompAutoField.RIGHT_TRENCH_START;
    Translation2d midPoint = CompAutoField.offset(startPose, -1.4, 0.0).getTranslation();
    Pose2d endPose = CompAutoField.offsetWithHeading(startPose, -2.2, 0.25, 165.0);
    var straightSegment =
        Trailblazer.segment("roborio-warmup-straight")
            .stopAt(CompAutoField.RIGHT_TRENCH_EXIT)
            .build();
    var curvedSegment =
        Trailblazer.segment("roborio-warmup")
            .maxSpeed(3.0)
            .through(midPoint, 0.25, goal -> goal.faceDegrees(175.0))
            .stopAt(endPose)
            .build();

    var segments =
        new com.team581.trailblazer3.model.AutoSegment[] {straightSegment, curvedSegment};
    for (int pass = 0; pass < 10; pass++) {
      var segment = segments[pass % segments.length];
      var prepared =
          warmupPlanner.prepare(segment, CompAutoField.clearance(Alliance.Blue), startPose);
      warmupPlanner.previewLaunch(prepared);
      warmupPlanner.activate(prepared);
      warmupPlanner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING);

      Pose2d pose = startPose;
      ChassisSpeeds speeds = new ChassisSpeeds();
      for (int i = 0; i < 150; i++) {
        syntheticClock[0] += i == 30 ? 0.15 : dt;
        var tickResult = warmupPlanner.update(syntheticClock[0], pose, speeds);
        warmupPlanner.currentTargetPose();
        speeds = tickResult.commandedFieldSpeeds();
        pose =
            new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * dt,
                pose.getY() + speeds.vyMetersPerSecond * dt,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
      }
      warmupPlanner.cancel();
    }
    Swerve.warmUpTrajectoryRequest();
  }

  private final Executor pathExecutor;

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;

  private final AutoTelemetry telemetry;
  private final CompletableFuture<Void> warmupFuture;

  private Alliance autoAlliance = currentAlliance();
  private SelectedAuto selectedAuto;
  private ChassisSpeeds stagedLaunch = new ChassisSpeeds();
  private boolean hasEnabledAuto;
  private boolean wasAutonomousEnabled;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    this(robotManager, trailblazer, createPathExecutor(), null);
  }

  Autos(
      RobotManager robotManager,
      Trailblazer trailblazer,
      Executor pathExecutor,
      @Nullable CompletableFuture<Void> warmupFuture) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.pathExecutor = pathExecutor;
    this.telemetry = new AutoTelemetry(trailblazer);
    selectedAuto =
        new SelectedAuto(
            AutoSelection.DO_NOTHING, autoAlliance, createAuto(AutoSelection.DO_NOTHING));
    this.warmupFuture =
        warmupFuture != null
            ? warmupFuture
            : CompletableFuture.runAsync(Autos::warmUpTrailblazer, pathExecutor);
  }

  private TrailblazerAuto<?> createAuto(AutoSelection state) {
    return state
        .create(new AutoContext(robotManager, trailblazer, autoAlliance, pathExecutor))
        .orElseThrow();
  }

  private boolean matchesSelection(AutoSelection state) {
    return selectedAuto.selection() == state && selectedAuto.alliance() == autoAlliance;
  }

  private void prepareWhileOutOfAuto(AutoSelection state) {
    autoAlliance = currentAlliance();

    boolean transitionedFromAuto = wasAutonomousEnabled;
    if (transitionedFromAuto) {
      selectAuto(AutoSelection.DO_NOTHING);
      wasAutonomousEnabled = false;
      telemetry.endRun();
      DogLog.log("Trailblazer3/AutoEnableObserved", false);
    }

    if (!matchesSelection(state)) {
      selectAuto(state);
    }
    var auto = selectedAuto.auto();
    var expectedStart = auto.startPose();
    var actualPose = robotManager.localization.getPose();
    if (DriverStation.isDisabled()
        && !hasEnabledAuto
        && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
        && (RobotBase.isSimulation()
            || DriverStation.isAutonomous()
            || DriverStation.isFMSAttached())) {
      actualPose = expectedStart;
      robotManager.localization.resetPose(actualPose);
    }

    boolean disabled = DriverStation.isDisabled();
    boolean autonomous = DriverStation.isAutonomous();
    boolean stageLaunch =
        disabled
            && autonomous
            && isStationaryForLaunch(robotManager.swerve.getFieldRelativeSpeeds());
    if (disabled) {
      auto.prepareWhileDisabled(actualPose, autonomous, stageLaunch);
    }
    updateStagedLaunch(stageLaunch ? auto.stagedLaunch(actualPose) : new ChassisSpeeds());
    telemetry.logIdle(state, auto, actualPose, transitionedFromAuto);
  }

  private void selectAuto(AutoSelection state) {
    trailblazer.cancel();
    selectedAuto.auto().discardPreparation();
    selectedAuto = new SelectedAuto(state, autoAlliance, createAuto(state));
  }

  private void updateFaults(TrailblazerAuto<?> auto) {
    if (!warmupFuture.isDone()) {
      DogLog.logFault(WARMUP_RUNNING_FAULT, AlertType.kWarning);
    } else {
      DogLog.clearFault(WARMUP_RUNNING_FAULT);
    }

    if (auto.hasPendingPathCalculations()) {
      DogLog.logFault(PATH_CALCULATION_RUNNING_FAULT, AlertType.kWarning);
    } else {
      DogLog.clearFault(PATH_CALCULATION_RUNNING_FAULT);
    }
  }

  private void updateStagedLaunch(ChassisSpeeds launch) {
    boolean stopped =
        launch.vxMetersPerSecond == 0
            && launch.vyMetersPerSecond == 0
            && launch.omegaRadiansPerSecond == 0;
    if (stopped) {
      robotManager.swerve.clearTrajectoryCommand();
    } else {
      robotManager.swerve.setDirectTrajectoryCommand(launch, TrajectoryOwner.STAGED);
    }
    if (!launch.equals(stagedLaunch)) {
      DogLog.log("Trailblazer3/StagedLaunchFieldSpeeds", launch);
      stagedLaunch = launch;
    }
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    return DriverStation.isEnabled() ? currentState : autoChooser.getSelectedAuto();
  }

  @Override
  protected void whileInState(AutoSelection state) {
    if (!DriverStation.isAutonomousEnabled()) {
      prepareWhileOutOfAuto(state);
      telemetry.logPlannedPath();
      updateFaults(selectedAuto.auto());
      return;
    }

    boolean firstTick = !wasAutonomousEnabled;
    if (firstTick) {
      autoAlliance = currentAlliance();
      if (!matchesSelection(state)) {
        selectAuto(state);
      }
      if (!selectedAuto.auto().hasQueuedPreparation()
          && !hasEnabledAuto
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && RobotBase.isSimulation()) {
        robotManager.localization.resetPose(selectedAuto.auto().startPose());
      }
      DogLog.log("Trailblazer3/AutoEnableObserved", true);
      robotManager.localization.consumePoseReset();
      wasAutonomousEnabled = true;
    }
    hasEnabledAuto = true;

    var auto = selectedAuto.auto();
    long stateStart = LoopTiming.start();
    auto.periodic();
    LoopTiming.end("Scheduler/Autos/StateMachine", stateStart);
    updateFaults(auto);

    var pose = robotManager.localization.getPose();
    var measuredSpeeds = robotManager.swerve.getFieldRelativeSpeeds();
    long followerStart = LoopTiming.start();
    double plannerTime = Utils.getCurrentTimeSeconds();
    TickResult result = trailblazer.update(plannerTime, pose, measuredSpeeds);
    LoopTiming.end("Scheduler/Autos/FollowerUpdate", followerStart);
    var crossingState = robotManager.localization.imu.bumpCrossingTracker.getState();
    boolean bumpOverride =
        auto.canDrive() && AutoActions.hasBumpCrossingOverride(result, crossingState);
    var command =
        bumpOverride
            ? AutoActions.bumpCrossingCommand(result, crossingState, autoAlliance)
            : result.commandedFieldSpeeds();
    // Apply the first endpoint directly so the disabled launch never ramps back from rest.
    if (firstTick || bumpOverride) {
      robotManager.swerve.setDirectTrajectoryCommand(
          command, bumpOverride ? TrajectoryOwner.BUMP : TrajectoryOwner.FIRST_ENABLED);
    } else {
      robotManager.swerve.setTrajectoryCommand(command, result.velocityStep());
    }
    long telemetryStart = LoopTiming.start();
    telemetry.logActive(state, auto, result, command, measuredSpeeds, firstTick);
    LoopTiming.end("Scheduler/Autos/Telemetry", telemetryStart);
  }
}
