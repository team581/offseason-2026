package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.google.common.collect.ImmutableMap;
import com.team581.autos.AutoChooser;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.state_machines.StateMachineSubsystem;
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
  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private final TrailblazerCalibrationAuto calibration;
  private final AutoTelemetry telemetry;

  private Alliance autoAlliance = Alliance.Blue;
  private @Nullable AutoSelection previewSelection;
  private @Nullable Alliance previewAlliance;
  private @Nullable TrailblazerAuto previewAuto;
  private @Nullable TrailblazerAuto stagedAuto;

  private boolean hasEnabledAuto = false;
  private boolean wasAutonomousEnabled = false;
  private @Nullable TrailblazerAuto activeAuto;

  public Autos(RobotManager robotManager, Trailblazer trailblazer, Vision vision) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.telemetry = new AutoTelemetry(trailblazer);
    this.calibration = new TrailblazerCalibrationAuto(robotManager, vision);
  }

  private TrailblazerAuto createAuto(AutoSelection state) {
    return state.create(new AutoContext(robotManager, trailblazer, autoAlliance)).orElseThrow();
  }

  private @Nullable TrailblazerAuto matchingPreview(AutoSelection state) {
    return previewSelection == state && previewAlliance == autoAlliance ? previewAuto : null;
  }

  private TrailblazerAuto prepareAuto(AutoSelection state) {
    var preview = matchingPreview(state);
    if (preview != null) {
      return preview;
    }
    preview = createAuto(state);
    previewSelection = state;
    previewAlliance = autoAlliance;
    previewAuto = preview;
    telemetry.publishPreview(preview, robotManager.localization.getPose());
    return preview;
  }

  private void prepareWhileOutOfAuto(AutoSelection state) {
    calibration.end();
    autoAlliance = DriverStation.getAlliance().orElse(autoAlliance);

    boolean transitionedFromAuto = wasAutonomousEnabled;
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
      telemetry.endRun();
    }

    if (TrailblazerCalibrationAuto.Test.fromSelection(state) != null) {
      stagedAuto = null;
      previewAuto = null;
      robotManager.swerve.clearTrajectoryCommand();
      telemetry.logCalibrationIdle(state, transitionedFromAuto);
      return;
    }

    var auto = prepareAuto(state);
    var expectedStart = auto.startPose();
    var actualPose = robotManager.localization.getPose();
    if (DriverStation.isDisabled()
        && !hasEnabledAuto
        && expectedStart.isPresent()
        && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
        && (RobotBase.isSimulation()
            || DriverStation.isAutonomous()
            || DriverStation.isFMSAttached())) {
      actualPose = expectedStart.orElseThrow();
      robotManager.localization.resetPose(actualPose);
    }

    if (!DriverStation.isDisabled() || !DriverStation.isAutonomous()) {
      stagedAuto = null;
      robotManager.swerve.clearTrajectoryCommand();
    } else if (!auto.equals(stagedAuto)) {
      stagedAuto = auto;
      robotManager.swerve.setTrajectoryCommand(
          auto.initialDisabledWheelTarget(actualPose).orElseGet(ChassisSpeeds::new));
    }

    telemetry.logIdle(state, auto, actualPose, transitionedFromAuto);
  }

  private TickResult updateTrailblazerForActiveTick(double now, RobotStateSnapshot snapshot) {
    var lastResult = trailblazer.lastResult();
    return switch (lastResult.status()) {
      case RUNNING, AT_GOAL, FAULT ->
          trailblazer.update(
              new TickInput(now, snapshot, ImmutableMap.of(), ImmutableMap.of(), Optional.empty()));
      case CANCELLED, DISABLED -> lastResult;
    };
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

    hasEnabledAuto = true;
    wasAutonomousEnabled = true;
    var calibrationTest = TrailblazerCalibrationAuto.Test.fromSelection(state);
    if (calibrationTest != null) {
      calibration.run(calibrationTest);
      return;
    }

    boolean firstTick = activeAuto == null;
    if (firstTick) {
      // Discard any pre-match disabled resetPose calls before autonomous begins.
      robotManager.localization.consumePoseReset();
      var preview = matchingPreview(state);
      activeAuto = preview != null ? preview : createAuto(state);
      previewAuto = null;
      stagedAuto = null;
    }
    if (firstTick || !trailblazer.isFaulted()) {
      activeAuto.periodic();
    }

    // Sample odometry after step/segment activation so setup work cannot age tick-0 timestamps.
    var drive = robotManager.swerve.drivetrain.getStateCopy();
    var pose = drive.Pose;
    var measuredSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.Speeds, pose.getRotation());
    double now = Utils.getCurrentTimeSeconds();

    boolean odometryReady =
        robotManager.swerve.isOdometryValid()
            && drive.SuccessfulDaqs > 0
            && drive.Timestamp > 0.0
            && Double.isFinite(drive.Timestamp);
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
    robotManager.swerve.setTrajectoryCommand(result.commandedFieldSpeeds());
    telemetry.logActive(state, activeAuto, result, snapshot, now, firstTick);
  }
}
