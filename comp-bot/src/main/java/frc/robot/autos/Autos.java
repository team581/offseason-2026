package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.autos.AutoChooser;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FmsUtil;
import com.team581.util.profiling.LoopTiming;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private record Preview(AutoSelection selection, Alliance alliance, TrailblazerAuto<?> auto) {}

  private static final int WARMUP_TICKS = 32;

  private static final double WARMUP_PERIOD_SECONDS = 0.020;

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

  private Optional<Preview> preview = Optional.empty();
  private Optional<TrailblazerAuto<?>> stagedAuto = Optional.empty();

  private boolean hasEnabledAuto = false;
  private boolean wasAutonomousEnabled = false;
  private Optional<TrailblazerAuto<?>> activeAuto = Optional.empty();

  private @Nullable Trailblazer warmupFollower;
  private Pose2d warmupPose = Pose2d.kZero;
  private ChassisSpeeds warmupSpeeds = new ChassisSpeeds();
  private int warmupTicks;

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

  private Optional<TrailblazerAuto<?>> matchingPreview(AutoSelection state) {
    return preview
        .filter(p -> p.selection() == state && p.alliance() == autoAlliance)
        .map(Preview::auto)
        .filter(auto -> !auto.hasStartedRunning());
  }

  private TrailblazerAuto<?> prepareAuto(AutoSelection state) {
    return matchingPreview(state)
        .orElseGet(() -> {
          var auto = createAuto(state);
          preview = Optional.of(new Preview(state, autoAlliance, auto));
          Pose2d startForPreview = auto.startPose().orElseGet(robotManager.localization::getPose);
          telemetry.publishPreview(auto, startForPreview);
          return auto;
        });
  }

  private void prepareWhileOutOfAuto(AutoSelection state) {
    calibration.end();
    autoAlliance = currentAlliance();

    boolean transitionedFromAuto = wasAutonomousEnabled || activeAuto.isPresent();
    if (transitionedFromAuto) {
      trailblazer.cancel();
      activeAuto = Optional.empty();
      wasAutonomousEnabled = false;
      stagedAuto = Optional.empty();
      telemetry.endRun();
    }

    if (TrailblazerCalibrationAuto.Test.fromSelection(state).isPresent()) {
      stagedAuto = Optional.empty();
      preview = Optional.empty();
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
      stagedAuto = Optional.empty();
    }

    if (!DriverStation.isDisabled() || !DriverStation.isAutonomous()) {
      stagedAuto = Optional.empty();
      robotManager.swerve.clearTrajectoryCommand();
    } else if (stagedAuto.filter(auto::equals).isEmpty()) {
      stagedAuto = Optional.of(auto);
      auto.initialDisabledWheelTarget(actualPose)
          .ifPresentOrElse(
              robotManager.swerve::setTrajectoryCommand,
              robotManager.swerve::clearTrajectoryCommand);
    }

    telemetry.logIdle(state, auto, actualPose, transitionedFromAuto);
    if (DriverStation.isDisabled() && RobotBase.isReal()) {
      warmUpFollower(auto, expectedStart.orElse(actualPose));
    }
  }

  private void warmUpFollower(TrailblazerAuto<?> auto, Pose2d start) {
    if (warmupTicks >= WARMUP_TICKS) {
      return;
    }
    var follower = warmupFollower;
    if (follower == null) {
      var segment = auto.initialPreviewSegment();
      if (segment.isEmpty()) {
        return;
      }
      follower = CompTrailblazer.create();
      follower.activate(segment.orElseThrow(), CompAutoField.clearance(autoAlliance));
      warmupFollower = follower;
      warmupPose = start;
      warmupSpeeds = new ChassisSpeeds();
    }

    // Exercise one control update per disabled loop on an isolated follower.
    long startNanos = LoopTiming.start();
    try {
      warmupSpeeds =
          follower
              .update(warmupTicks * WARMUP_PERIOD_SECONDS, warmupPose, warmupSpeeds)
              .commandedFieldSpeeds();
    } catch (IllegalArgumentException exception) {
      DogLog.log("Trailblazer3/WarmupError", exception.toString());
      warmupTicks = WARMUP_TICKS;
      warmupFollower = null;
      return;
    } finally {
      LoopTiming.end("Scheduler/Autos/DisabledWarmupUpdate", startNanos);
    }
    warmupPose =
        warmupPose.exp(
            ChassisSpeeds.fromFieldRelativeSpeeds(warmupSpeeds, warmupPose.getRotation())
                .toTwist2d(WARMUP_PERIOD_SECONDS));
    if (++warmupTicks == WARMUP_TICKS) {
      warmupFollower = null;
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

    boolean firstTick = !wasAutonomousEnabled || activeAuto.isEmpty();
    if (firstTick) {
      autoAlliance = currentAlliance();
      var prepared = matchingPreview(state);
      activeAuto = Optional.of(prepared.orElseGet(() -> createAuto(state)));
      preview = Optional.empty();
      stagedAuto = Optional.empty();
      if (prepared.isEmpty()
          && !hasEnabledAuto
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && RobotBase.isSimulation()) {
        activeAuto.orElseThrow().startPose().ifPresent(robotManager.localization::resetPose);
      }
      robotManager.localization.consumePoseReset();
      wasAutonomousEnabled = true;
    }
    hasEnabledAuto = true;

    var auto = activeAuto.orElseThrow();
    long stateStart = LoopTiming.start();
    auto.periodic();
    LoopTiming.end("Scheduler/Autos/StateMachine", stateStart);
    if (!firstTick && robotManager.localization.consumePoseReset()) {
      trailblazer.relatch();
    }

    var pose = robotManager.localization.getPose();
    var measuredSpeeds = robotManager.swerve.getFieldRelativeSpeeds();
    long followerStart = LoopTiming.start();
    TickResult result = trailblazer.update(Utils.getCurrentTimeSeconds(), pose, measuredSpeeds);
    LoopTiming.end("Scheduler/Autos/FollowerUpdate", followerStart);
    var command =
        AutoActions.applyBumpCrossingOverride(
            result, robotManager.localization.imu.bumpCrossingTracker.getState(), autoAlliance);
    robotManager.swerve.setTrajectoryCommand(command);
    long telemetryStart = LoopTiming.start();
    telemetry.logActive(state, auto, result, command, measuredSpeeds, firstTick);
    LoopTiming.end("Scheduler/Autos/Telemetry", telemetryStart);
  }
}
