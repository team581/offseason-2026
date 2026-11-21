package frc.robot.autos;

import com.team581.math.MathHelpers;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jspecify.annotations.Nullable;

final class AutoTelemetry {
  private static void logSpeeds(ChassisSpeeds command, ChassisSpeeds measured) {
    double commandedSpeed = MathHelpers.getLinearVelocity(command);
    double measuredSpeed = MathHelpers.getLinearVelocity(measured);
    if (Double.isFinite(commandedSpeed)) {
      DogLog.log("Trailblazer3/CommandedSpeed", commandedSpeed, "m/s");
    }
    if (Double.isFinite(measuredSpeed)) {
      DogLog.log("Trailblazer3/MeasuredSpeed", measuredSpeed, "m/s");
    }
    if (Double.isFinite(command.omegaRadiansPerSecond)) {
      DogLog.log("Trailblazer3/CommandedYawRate", command.omegaRadiansPerSecond, "rad/s");
    }
    if (Double.isFinite(measured.omegaRadiansPerSecond)) {
      DogLog.log("Trailblazer3/MeasuredYawRate", measured.omegaRadiansPerSecond, "rad/s");
    }
  }

  private final Trailblazer trailblazer;
  private AutoSelection lastLoggedSelection = AutoSelection.DO_NOTHING;
  private String lastLoggedStep = "";
  private @Nullable RuntimeStatus lastLoggedStatus;
  private @Nullable LimitingFactor lastLoggedLimitingFactor;
  private boolean lastNearStart;
  private long lastLoggedGeneration = -1;
  private String lastLoggedDiagnostic = "";
  private boolean lastLoggedInfeasible;

  AutoTelemetry(Trailblazer trailblazer) {
    this.trailblazer = trailblazer;
    warmUp();
  }

  private void warmUp() {
    DogLog.log("Autos/SelectedAutoName", lastLoggedSelection.toString());
    DogLog.log("Autos/Step", "IDLE");
    DogLog.log("Autos/AtExpectedStartPose", false);
    DogLog.log("Trailblazer3/Status", RuntimeStatus.IDLE.name());
    DogLog.log("Trailblazer3/CommandFieldSpeeds", new ChassisSpeeds());
    logSpeeds(new ChassisSpeeds(), new ChassisSpeeds());
    DogLog.log("Trailblazer3/LimitingFactor", LimitingFactor.NONE.name());
    DogLog.log("Trailblazer3/TargetPose", Pose2d.kZero);
    DogLog.log("Trailblazer3/PlannedPath", new Pose2d[0]);
    DogLog.log("Trailblazer3/Diagnostic", "");
    DogLog.log("Trailblazer3/Infeasible", false);
    DogLog.log("Trailblazer3/AutoEnableObserved", false);
    DogLog.log("Trailblazer3/StagedLaunchFieldSpeeds", new ChassisSpeeds());
    DogLog.log("Trailblazer3/WaitingForPath", "");
    DogLog.log("Trailblazer3/PathPreparationFailure", "");
    DogLog.log("Scheduler/Autos/StateMachine", 0.0);
    DogLog.log("Scheduler/Autos/FollowerUpdate", 0.0);
    DogLog.log("Scheduler/Autos/Telemetry", 0.0);
  }

  void endRun() {
    lastLoggedStep = "";
    lastLoggedStatus = null;
    lastLoggedLimitingFactor = null;
    lastLoggedDiagnostic = "";
    lastLoggedInfeasible = false;
  }

  void logActive(
      AutoSelection state,
      TrailblazerAuto<?> auto,
      TickResult result,
      ChassisSpeeds command,
      ChassisSpeeds measuredSpeeds,
      boolean firstTick) {
    if (firstTick) {
      lastLoggedStep = "";
      lastLoggedStatus = null;
      lastLoggedLimitingFactor = null;
      lastLoggedDiagnostic = "";
      lastLoggedInfeasible = false;
    }
    if (firstTick || lastLoggedSelection != state) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      lastLoggedSelection = state;
    }

    var step = auto.currentStepName();
    if (!step.equals(lastLoggedStep)) {
      DogLog.log("Autos/Step", step);
      lastLoggedStep = step;
    }

    if (result.status() != lastLoggedStatus) {
      DogLog.log("Trailblazer3/Status", result.status().name());
      lastLoggedStatus = result.status();
    }

    if (result.status() == RuntimeStatus.RUNNING) {
      DogLog.log("Trailblazer3/CommandFieldSpeeds", command);
      logSpeeds(command, measuredSpeeds);
      if (result.limitingFactor() != lastLoggedLimitingFactor) {
        DogLog.log("Trailblazer3/LimitingFactor", result.limitingFactor().name());
        lastLoggedLimitingFactor = result.limitingFactor();
      }
      trailblazer
          .currentTargetPose()
          .ifPresent(target -> DogLog.log("Trailblazer3/TargetPose", target));
    }
    logPlannedPath();
    if (!result.diagnostic().equals(lastLoggedDiagnostic)) {
      DogLog.log("Trailblazer3/Diagnostic", result.diagnostic());
      lastLoggedDiagnostic = result.diagnostic();
    }
    if (result.infeasible() != lastLoggedInfeasible) {
      DogLog.log("Trailblazer3/Infeasible", result.infeasible());
      lastLoggedInfeasible = result.infeasible();
    }
  }

  void logIdle(
      AutoSelection state,
      TrailblazerAuto<?> auto,
      Pose2d actualPose,
      boolean transitionedFromAuto) {
    boolean nearStart = auto.isNearStartPose(actualPose);
    if (transitionedFromAuto || lastLoggedSelection != state || lastNearStart != nearStart) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      if (!"IDLE".equals(lastLoggedStep)) {
        DogLog.log("Autos/Step", "IDLE");
        lastLoggedStep = "IDLE";
      }
      DogLog.log("Autos/AtExpectedStartPose", nearStart);
      lastLoggedSelection = state;
      lastNearStart = nearStart;
    }
  }

  void logPlannedPath() {
    long generation = trailblazer.pathGeneration();
    if (generation != lastLoggedGeneration) {
      lastLoggedGeneration = generation;
      DogLog.log(
          "Trailblazer3/PlannedPath", trailblazer.sampledPathPoses(Trailblazer.PATH_LOG_SPACING));
    }
  }
}
