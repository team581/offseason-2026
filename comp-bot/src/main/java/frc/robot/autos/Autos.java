package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSet;
import com.team581.autos.AutoChooser;
import com.team581.math.MathHelpers;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.profiling.DiagnosticCadence;
import com.team581.util.profiling.LoopTiming;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.Optional;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private static final String DIAGNOSTIC_KEY = "Trailblazer3/Diagnostic";
  private static final Pose2d[] EMPTY_PATH = new Pose2d[0];

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private final TrailblazerCalibrationAuto calibration;

  private Alliance autoAlliance = Alliance.Blue;
  private @Nullable AutoSelection previewSelection;
  private @Nullable Alliance previewAlliance;
  private @Nullable TrailblazerAuto previewAuto;
  private @Nullable TrailblazerAuto stagedAuto;

  private boolean hasEnabledAuto = false;
  private boolean wasAutonomousEnabled = false;
  private @Nullable TrailblazerAuto activeAuto;

  private double previousMeasuredSpeed;
  private double previousMeasurementTime;
  private boolean hasPreviousMeasuredSpeed;
  private boolean braking;

  private @Nullable AutoSelection lastLoggedSelection;
  private boolean lastNearStart;
  private @Nullable RuntimeStatus lastLoggedStatus;
  private long lastLoggedGeneration = -1;

  public Autos(RobotManager robotManager, Trailblazer trailblazer, Vision vision) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.calibration = new TrailblazerCalibrationAuto(robotManager, vision);
  }

  private void ensurePreviewAuto(AutoSelection state, Alliance currentAlliance) {
    if (previewAuto == null
        || previewSelection != state
        || previewAlliance != currentAlliance
        || previewAuto.hasStartedRunning()) {
      previewSelection = state;
      previewAlliance = currentAlliance;
      previewAuto =
          state.create(new AutoContext(robotManager, trailblazer, currentAlliance)).orElse(null);
      Pose2d startForPreview =
          previewAuto == null
              ? robotManager.localization.getPose()
              : previewAuto.startPose().orElseGet(robotManager.localization::getPose);
      DogLog.log(
          "Autos/PreviewPath",
          previewAuto == null ? EMPTY_PATH : previewAuto.previewPath(startForPreview));
    }
  }

  private void handleNotAutonomousEnabled(AutoSelection state) {
    calibration.end();
    autoAlliance = DriverStation.getAlliance().orElse(autoAlliance);

    boolean transitionedFromAuto = wasAutonomousEnabled || activeAuto != null;
    if (transitionedFromAuto) {
      trailblazer.preempt();
      trailblazer.update(
          RobotStateSnapshot.measured(
              Utils.getCurrentTimeSeconds(),
              robotManager.swerve.getDriveState().Pose,
              new ChassisSpeeds(),
              false));
      activeAuto = null;
      wasAutonomousEnabled = false;
      lastLoggedGeneration = -1;
      DogLog.log("Trailblazer3/PlannedPath", EMPTY_PATH);
    }

    if (TrailblazerCalibrationAuto.Test.fromSelection(state) != null) {
      stagedAuto = null;
      previewAuto = null;
      previewSelection = state;
      previewAlliance = autoAlliance;
      robotManager.swerve.clearTrajectoryCommand();
      if (transitionedFromAuto
          || lastLoggedSelection != state
          || DiagnosticCadence.shouldLogRoutine()) {
        if (lastLoggedSelection != state) {
          DogLog.log("Autos/PreviewPath", EMPTY_PATH);
        }
        DogLog.log("Autos/SelectedAutoName", state.toString());
        DogLog.log("Autos/Step", "CALIBRATION_IDLE");
        DogLog.log("Autos/StepIndex", -1);
        DogLog.log("Autos/Finished", false);
        DogLog.log("Autos/HasStartPose", false);
        DogLog.log("Autos/ExpectedStartPose", Pose2d.kZero);
        DogLog.log("Autos/AtExpectedStartPose", true);
        DogLog.log("Autos/StartPoseWarning", false);
        DogLog.log("Autos/StartPoseDistanceMeters", 0.0);
        DogLog.log("Autos/StartPoseHeadingErrorDeg", 0.0);
        lastLoggedSelection = state;
        lastNearStart = true;
        lastLoggedStatus = null;
      }
      return;
    }

    ensurePreviewAuto(state, autoAlliance);
    if (previewAuto == null) {
      robotManager.swerve.clearTrajectoryCommand();
      return;
    }

    var expectedStart = previewAuto.startPose();
    var actualPose = robotManager.localization.getPose();
    if (DriverStation.isDisabled()) {
      if (!hasEnabledAuto
          && expectedStart.isPresent()
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && (RobotBase.isSimulation()
              || DriverStation.isAutonomous()
              || DriverStation.isFMSAttached())) {
        actualPose = expectedStart.orElseThrow();
        robotManager.localization.resetPose(actualPose);
      }
      if (!DriverStation.isAutonomous()) {
        stagedAuto = null;
        robotManager.swerve.clearTrajectoryCommand();
      } else if (!previewAuto.equals(stagedAuto)) {
        stagedAuto = previewAuto;
        robotManager.swerve.setTrajectoryCommand(
            previewAuto.initialDisabledWheelTarget(actualPose).orElseGet(ChassisSpeeds::new));
      }
    } else {
      stagedAuto = null;
      robotManager.swerve.clearTrajectoryCommand();
    }

    boolean nearStart = previewAuto.isNearStartPose(actualPose);
    if (transitionedFromAuto
        || lastLoggedSelection != state
        || lastNearStart != nearStart
        || DiagnosticCadence.shouldLogRoutine()) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      DogLog.log("Autos/Step", "IDLE");
      DogLog.log("Autos/StepIndex", -1);
      DogLog.log("Autos/Finished", false);
      DogLog.log("Autos/HasStartPose", expectedStart.isPresent());
      DogLog.log("Autos/ExpectedStartPose", expectedStart.orElse(Pose2d.kZero));
      DogLog.log("Autos/AtExpectedStartPose", nearStart);
      DogLog.log("Autos/StartPoseWarning", expectedStart.isPresent() && !nearStart);
      if (expectedStart.isPresent()) {
        var expected = expectedStart.orElseThrow();
        DogLog.log(
            "Autos/StartPoseDistanceMeters",
            actualPose.getTranslation().getDistance(expected.getTranslation()));
        DogLog.log(
            "Autos/StartPoseHeadingErrorDeg",
            Math.abs(actualPose.getRotation().minus(expected.getRotation()).getDegrees()));
      } else {
        DogLog.log("Autos/StartPoseDistanceMeters", 0.0);
        DogLog.log("Autos/StartPoseHeadingErrorDeg", 0.0);
      }
      lastLoggedSelection = state;
      lastNearStart = nearStart;
      lastLoggedStatus = null;
    }
  }

  private void logAutonomousTelemetry(
      TickResult result, RobotStateSnapshot snapshot, boolean forceFullLog) {
    boolean logRoutine = forceFullLog || DiagnosticCadence.shouldLogRoutine();
    DogLog.log("Autos/Step", activeAuto.currentStepName());
    DogLog.log("Autos/StepIndex", activeAuto.currentStepIndex());
    DogLog.log("Autos/Finished", activeAuto.isFinished());

    DogLog.log("Trailblazer3/Status", result.status().name());
    DogLog.log("Trailblazer3/Infeasible", result.infeasible());
    DogLog.log("Trailblazer3/Progress", result.progress());
    if (result.geometryGeneration() != lastLoggedGeneration) {
      lastLoggedGeneration = result.geometryGeneration();
      DogLog.log("Trailblazer3/PlannedPath", trailblazer.sampledPathPoses(0.10));
    }

    double measuredSpeed = MathHelpers.getLinearVelocity(snapshot.measuredFieldSpeeds());
    double measurementTime = snapshot.localizationValid() ? snapshot.poseTimestampSeconds() : 0.0;
    DogLog.log("Trailblazer3/MeasuredSpeedMps", measuredSpeed);
    if (hasPreviousMeasuredSpeed && measurementTime > previousMeasurementTime) {
      DogLog.log(
          "Trailblazer3/MeasuredAccelerationMps2",
          (measuredSpeed - previousMeasuredSpeed) / (measurementTime - previousMeasurementTime));
    }
    if (measurementTime > 0) {
      previousMeasuredSpeed = measuredSpeed;
      previousMeasurementTime = measurementTime;
      hasPreviousMeasuredSpeed = true;
    }

    var command = result.commandedFieldSpeeds();
    DogLog.log("Trailblazer3/CommandSpeedMps", MathHelpers.getLinearVelocity(command));

    var maybeMotion = trailblazer.motionTelemetry();
    if (maybeMotion.isPresent()) {
      var motion = maybeMotion.orElseThrow();
      if (!braking && motion.commandedAcceleration() < -1e-6) {
        braking = true;
        DogLog.log("Trailblazer3/BrakingStartMeters", result.progress());
      }
      if (logRoutine) {
        DogLog.log("Trailblazer3/ProfileAccelerationMps2", motion.profileAcceleration());
        DogLog.log("Trailblazer3/CommandAccelerationMps2", motion.commandedAcceleration());
        DogLog.log("Trailblazer3/AvailableAccelerationMps2", motion.availableAcceleration());
        DogLog.log("Trailblazer3/ProfileSpeedMps", motion.profileSpeed());
        DogLog.log("Trailblazer3/MaxSpeedMps", motion.maxSpeed());
        DogLog.log("Trailblazer3/MaxBrakingMps2", motion.maxBraking());
        DogLog.log("Trailblazer3/MaxAngularSpeedRadPerSec", motion.maxAngularSpeed());
        DogLog.log(
            "Trailblazer3/MaxAngularAccelerationRadPerSec2", motion.maxAngularAcceleration());
        DogLog.log("Trailblazer3/DistanceRemainingMeters", motion.distanceRemaining());
        DogLog.log("Trailblazer3/PositionErrorMeters", motion.positionError());
        DogLog.log("Trailblazer3/StopDwellSeconds", motion.dwellSeconds());
      }
    }
    if (logRoutine) {
      DogLog.log("Trailblazer3/ProfileLimit", result.profileLimitingFactor().name());
      DogLog.log("Trailblazer3/CommandLimit", result.commandLimitingFactor().name());
      trailblazer
          .currentTargetPose()
          .ifPresent(target -> DogLog.log("Trailblazer3/TargetPose", target));
    }
    DogLog.log(DIAGNOSTIC_KEY, result.diagnostic());
  }

  private TickResult updateTrailblazerForActiveTick(double now, RobotStateSnapshot snapshot) {
    var status = trailblazer.lastResult().status();
    boolean plannerActive =
        status == RuntimeStatus.RUNNING
            || status == RuntimeStatus.AT_GOAL
            || status == RuntimeStatus.FAULT;
    if (!plannerActive) {
      return new TickResult(
          new ChassisSpeeds(),
          new TickResult.ChassisAccelerations(0, 0, 0),
          ImmutableList.of(),
          status,
          false,
          TickResult.LimitingFactor.NONE,
          TickResult.LimitingFactor.NONE,
          ImmutableSet.of(),
          ImmutableMap.of(),
          trailblazer.lastResult().geometryGeneration(),
          trailblazer.lastResult().progress(),
          "");
    }
    long updateStart = LoopTiming.start();
    try {
      return trailblazer.update(
          new TickInput(now, snapshot, ImmutableMap.of(), ImmutableMap.of(), Optional.empty()));
    } finally {
      LoopTiming.end("Scheduler/Autos/UpdatePlanner", updateStart);
    }
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    return DriverStation.isEnabled() ? currentState : autoChooser.getSelectedAuto();
  }

  @Override
  protected void whileInState(AutoSelection state) {
    if (!DriverStation.isAutonomousEnabled()) {
      handleNotAutonomousEnabled(state);
      return;
    }

    hasEnabledAuto = true;
    var calibrationTest = TrailblazerCalibrationAuto.Test.fromSelection(state);
    if (calibrationTest != null) {
      wasAutonomousEnabled = true;
      calibration.run(calibrationTest);
      return;
    }

    boolean firstTick = !wasAutonomousEnabled || activeAuto == null;
    long activationStart = firstTick ? LoopTiming.start() : 0;
    if (firstTick) {
      // Discard any pre-match disabled resetPose calls before autonomous begins.
      robotManager.localization.consumePoseReset();
      if (previewAuto != null
          && previewSelection == state
          && previewAlliance == autoAlliance
          && !previewAuto.hasStartedRunning()) {
        activeAuto = previewAuto;
      } else {
        activeAuto =
            state.create(new AutoContext(robotManager, trailblazer, autoAlliance)).orElseThrow();
      }
      previewAuto = null;
      hasPreviousMeasuredSpeed = false;
      braking = false;
      lastLoggedStatus = null;
      lastLoggedGeneration = -1;
      wasAutonomousEnabled = true;
    }
    if (firstTick || lastLoggedSelection != state || DiagnosticCadence.shouldLogRoutine()) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      lastLoggedSelection = state;
    }

    if (firstTick) {
      activeAuto.periodic();
      LoopTiming.end("Scheduler/Autos/Activate", activationStart);
    } else if (!trailblazer.isFaulted()) {
      activeAuto.periodic();
    }

    // Sample odometry after step/segment activation so setup work cannot age tick-0 timestamps.
    long odometryStart = LoopTiming.start();
    var drive = robotManager.swerve.drivetrain.getStateCopy();
    var pose = drive.Pose;
    var measuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.Speeds, pose.getRotation());
    double now = Utils.getCurrentTimeSeconds();
    LoopTiming.end("Scheduler/Autos/ReadOdometry", odometryStart);

    boolean odometryReady =
        robotManager.swerve.isOdometryValid()
            && drive.SuccessfulDaqs > 0
            && drive.Timestamp > 0.0
            && Double.isFinite(drive.Timestamp);
    DogLog.log(
        "Trailblazer3/OdometryAgeMs",
        odometryReady ? (now - drive.Timestamp) * 1000.0 : Double.NaN);
    DogLog.log("Trailblazer3/OdometryReady", odometryReady);
    boolean midRunPoseReset = !firstTick && robotManager.localization.consumePoseReset();
    var snapshot =
        new RobotStateSnapshot(
            odometryReady ? drive.Timestamp : 0.0,
            pose,
            measuredSpeeds,
            odometryReady,
            midRunPoseReset,
            Optional.of(robotManager.swerve.getLastAppliedFieldSpeeds()),
            OptionalDouble.empty(),
            OptionalDouble.empty(),
            true);

    TickResult result = updateTrailblazerForActiveTick(now, snapshot);
    long outputStart = LoopTiming.start();
    robotManager.swerve.setTrajectoryCommand(result.commandedFieldSpeeds());
    if (firstTick) {
      LoopTiming.end("Scheduler/Autos/ActivationThroughFirstOutput", activationStart);
    }

    boolean forceFullLog =
        firstTick
            || result.status() != lastLoggedStatus
            || result.infeasible()
            || result.status() != RuntimeStatus.RUNNING;
    lastLoggedStatus = result.status();
    logAutonomousTelemetry(result, snapshot, forceFullLog);
    LoopTiming.end("Scheduler/Autos/Output", outputStart);
  }
}
