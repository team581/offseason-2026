package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.autos.AutoChooser;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import org.jspecify.annotations.Nullable;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private static Alliance currentAlliance() {
    return FmsUtil.isRedAlliance() ? Alliance.Red : Alliance.Blue;
  }

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private final TrailblazerCalibrationAuto calibration;
  private final AutoTelemetry telemetry;

  private Alliance autoAlliance = currentAlliance();
  private @Nullable AutoSelection previewSelection;
  private @Nullable Alliance previewAlliance;
  private @Nullable TrailblazerAuto<?> previewAuto;
  private @Nullable TrailblazerAuto<?> stagedAuto;

  private boolean hasEnabledAuto = false;
  private boolean wasAutonomousEnabled = false;
  private @Nullable TrailblazerAuto<?> activeAuto;

  public Autos(RobotManager robotManager, Trailblazer trailblazer, Vision vision) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    this.telemetry = new AutoTelemetry(trailblazer);
    this.calibration = new TrailblazerCalibrationAuto(robotManager, vision);
  }

  private TrailblazerAuto<?> createAuto(AutoSelection state) {
    return state.create(new AutoContext(robotManager, trailblazer, autoAlliance)).orElseThrow();
  }

  private @Nullable TrailblazerAuto<?> matchingPreview(AutoSelection state) {
    return previewSelection == state
            && previewAlliance == autoAlliance
            && previewAuto != null
            && !previewAuto.hasStartedRunning()
        ? previewAuto
        : null;
  }

  private TrailblazerAuto<?> prepareAuto(AutoSelection state) {
    var preview = matchingPreview(state);
    if (preview != null) {
      return preview;
    }
    preview = createAuto(state);
    previewSelection = state;
    previewAlliance = autoAlliance;
    previewAuto = preview;
    Pose2d startForPreview = preview.startPose().orElseGet(robotManager.localization::getPose);
    telemetry.publishPreview(preview, startForPreview);
    return preview;
  }

  private void prepareWhileOutOfAuto(AutoSelection state) {
    calibration.end();
    autoAlliance = currentAlliance();

    boolean transitionedFromAuto = wasAutonomousEnabled || activeAuto != null;
    if (transitionedFromAuto) {
      trailblazer.cancel();
      activeAuto = null;
      wasAutonomousEnabled = false;
      stagedAuto = null;
      telemetry.endRun();
    }

    if (TrailblazerCalibrationAuto.Test.fromSelection(state) != null) {
      stagedAuto = null;
      previewAuto = null;
      previewSelection = state;
      previewAlliance = autoAlliance;
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
      stagedAuto = null;
    }

    if (!DriverStation.isDisabled() || !DriverStation.isAutonomous()) {
      stagedAuto = null;
      robotManager.swerve.clearTrajectoryCommand();
    } else if (!auto.equals(stagedAuto)) {
      stagedAuto = auto;
      auto.initialDisabledWheelTarget(actualPose)
          .ifPresentOrElse(
              robotManager.swerve::setTrajectoryCommand,
              robotManager.swerve::clearTrajectoryCommand);
    }

    telemetry.logIdle(state, auto, actualPose, transitionedFromAuto);
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
    if (calibrationTest != null) {
      hasEnabledAuto = true;
      wasAutonomousEnabled = true;
      calibration.run(calibrationTest);
      return;
    }

    boolean firstTick = !wasAutonomousEnabled || activeAuto == null;
    if (firstTick) {
      Alliance latestAlliance = currentAlliance();
      boolean allianceChanged = latestAlliance != autoAlliance;
      autoAlliance = latestAlliance;
      var preview = matchingPreview(state);
      activeAuto = preview != null ? preview : createAuto(state);
      previewAuto = null;
      stagedAuto = null;
      if (allianceChanged
          && !hasEnabledAuto
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && RobotBase.isSimulation()) {
        activeAuto.startPose().ifPresent(robotManager.localization::resetPose);
      }
      robotManager.localization.consumePoseReset();
      wasAutonomousEnabled = true;
    }
    hasEnabledAuto = true;

    activeAuto.periodic();
    if (!firstTick && robotManager.localization.consumePoseReset()) {
      trailblazer.relatch();
    }

    var pose = robotManager.localization.getPose();
    var measuredSpeeds = robotManager.swerve.getFieldRelativeSpeeds();
    TickResult result = trailblazer.update(Utils.getCurrentTimeSeconds(), pose, measuredSpeeds);
    robotManager.swerve.setTrajectoryCommand(result.commandedFieldSpeeds());
    telemetry.logActive(state, activeAuto, result, firstTick);
  }
}
