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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.Swerve.TrajectoryOwner;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private record Preview(AutoSelection selection, Alliance alliance, TrailblazerAuto<?> auto) {}

  private static Alliance currentAlliance() {
    return FmsUtil.isRedAlliance() ? Alliance.Red : Alliance.Blue;
  }

  private static boolean isStationaryForLaunch(ChassisSpeeds speeds) {
    return MathHelpers.getLinearVelocity(speeds) <= 0.10
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Math.toRadians(10.0));
  }

  private final Executor pathExecutor =
      Executors.newSingleThreadExecutor(task -> {
        var worker = new Thread(task, "trailblazer-path-planner");
        worker.setDaemon(true);
        return worker;
      });

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;

  private final TrailblazerCalibrationAuto calibration;
  private final AutoTelemetry telemetry;

  private Alliance autoAlliance = currentAlliance();
  private Preview preview;
  private ChassisSpeeds stagedLaunch = new ChassisSpeeds();
  private boolean hasEnabledAuto;
  private boolean wasAutonomousEnabled;

  public Autos(RobotManager robotManager, Trailblazer trailblazer, Vision vision) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.telemetry = new AutoTelemetry(trailblazer, robotManager.swerve);
    this.calibration = new TrailblazerCalibrationAuto(robotManager, vision);
    preview =
        new Preview(AutoSelection.DO_NOTHING, autoAlliance, createAuto(AutoSelection.DO_NOTHING));
  }

  private TrailblazerAuto<?> createAuto(AutoSelection state) {
    return state
        .create(new AutoContext(robotManager, trailblazer, autoAlliance, pathExecutor))
        .orElseThrow();
  }

  private boolean matchesPreview(AutoSelection state) {
    return preview.selection() == state && preview.alliance() == autoAlliance;
  }

  private void prepareWhileOutOfAuto(AutoSelection state) {
    calibration.end();
    autoAlliance = currentAlliance();

    boolean transitionedFromAuto = wasAutonomousEnabled;
    if (transitionedFromAuto) {
      trailblazer.cancel();
      selectAuto(AutoSelection.DO_NOTHING);
      wasAutonomousEnabled = false;
      telemetry.endRun();
      DogLog.log("Trailblazer3/AutoEnableObserved", false);
    }

    if (TrailblazerCalibrationAuto.Test.fromSelection(state).isPresent()) {
      if (!matchesPreview(AutoSelection.DO_NOTHING)) {
        selectAuto(AutoSelection.DO_NOTHING);
      }
      updateStagedLaunch(new ChassisSpeeds());
      telemetry.logCalibrationIdle(state, transitionedFromAuto);
      return;
    }

    if (!matchesPreview(state)) {
      selectAuto(state);
    }
    var auto = preview.auto();
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

    boolean stageLaunch =
        DriverStation.isDisabled()
            && DriverStation.isAutonomous()
            && isStationaryForLaunch(robotManager.swerve.getFieldRelativeSpeeds());
    if (DriverStation.isDisabled()) {
      auto.prepareWhileDisabled(actualPose, stageLaunch);
    }
    updateStagedLaunch(stageLaunch ? auto.stagedLaunch(actualPose) : new ChassisSpeeds());
    telemetry.logIdle(state, auto, actualPose, transitionedFromAuto);
    if (stageLaunch)
      telemetry.logStaged(state, stagedLaunch, auto.startPose().getRotation().getRadians());
  }

  private void selectAuto(AutoSelection state) {
    preview.auto().discardPreparation();
    preview = new Preview(state, autoAlliance, createAuto(state));
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
      return;
    }

    var calibrationTest = TrailblazerCalibrationAuto.Test.fromSelection(state);
    if (calibrationTest.isPresent()) {
      hasEnabledAuto = true;
      wasAutonomousEnabled = true;
      calibration.run(calibrationTest.orElseThrow());
      return;
    }

    boolean firstTick = !wasAutonomousEnabled;
    if (firstTick) {
      autoAlliance = currentAlliance();
      if (!matchesPreview(state)) {
        selectAuto(state);
      }
      if (!preview.auto().hasPreparedPreview()
          && !hasEnabledAuto
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && RobotBase.isSimulation()) {
        robotManager.localization.resetPose(preview.auto().startPose());
      }
      DogLog.log("Trailblazer3/AutoEnableObserved", true);
      robotManager.localization.consumePoseReset();
      wasAutonomousEnabled = true;
    }
    hasEnabledAuto = true;

    var auto = preview.auto();
    long stateStart = LoopTiming.start();
    auto.periodic();
    LoopTiming.end("Scheduler/Autos/StateMachine", stateStart);
    if (!firstTick && robotManager.localization.consumePoseReset()) {
      trailblazer.relatch();
    }

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
    telemetry.logActive(
        state, auto, result, command, measuredSpeeds, firstTick, plannerTime, bumpOverride);
    LoopTiming.end("Scheduler/Autos/Telemetry", telemetryStart);
  }
}
