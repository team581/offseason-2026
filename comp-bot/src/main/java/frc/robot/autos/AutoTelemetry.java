package frc.robot.autos;

import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.profiling.DiagnosticCadence;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jspecify.annotations.Nullable;

final class AutoTelemetry {
  private static final Pose2d[] EMPTY_PATH = new Pose2d[0];

  private final Trailblazer trailblazer;
  private @Nullable AutoSelection lastLoggedSelection;
  private boolean lastNearStart;
  private @Nullable RuntimeStatus lastLoggedStatus;
  private long lastLoggedGeneration = -1;
  private String lastLoggedDiagnostic = "";
  private boolean lastLoggedInfeasible = false;

  AutoTelemetry(Trailblazer trailblazer) {
    this.trailblazer = trailblazer;
  }

  void endRun() {
    lastLoggedGeneration = -1;
    lastLoggedStatus = null;
    lastLoggedDiagnostic = "";
    lastLoggedInfeasible = false;
    DogLog.log("Trailblazer3/PlannedPath", EMPTY_PATH);
  }

  void logActive(
      AutoSelection state,
      TrailblazerAuto<?> auto,
      TickResult result,
      ChassisSpeeds command,
      ChassisSpeeds measuredSpeeds,
      boolean firstTick) {
    if (firstTick) {
      lastLoggedStatus = null;
      lastLoggedGeneration = -1;
      lastLoggedDiagnostic = "";
      lastLoggedInfeasible = false;
    }
    if (firstTick || lastLoggedSelection != state) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      lastLoggedSelection = state;
    }

    DogLog.log("Autos/Step", auto.currentStepName());
    DogLog.log("Trailblazer3/Status", result.status().name());
    if (result.status() == RuntimeStatus.RUNNING) {
      DogLog.log("Trailblazer3/CommandFieldSpeeds", command);
      DogLog.log("Trailblazer3/MeasuredFieldSpeeds", measuredSpeeds);
      DogLog.log("Trailblazer3/LimitingFactor", result.limitingFactor().name());
    }
    if (result.geometryGeneration() != lastLoggedGeneration) {
      lastLoggedGeneration = result.geometryGeneration();
      DogLog.log("Trailblazer3/PlannedPath", trailblazer.sampledPathPoses(0.10));
    }
    if (DiagnosticCadence.shouldLogRoutine() || result.status() != lastLoggedStatus) {
      trailblazer
          .currentTargetPose()
          .ifPresent(target -> DogLog.log("Trailblazer3/TargetPose", target));
      lastLoggedStatus = result.status();
    }
    if (!result.diagnostic().equals(lastLoggedDiagnostic) || !result.diagnostic().isEmpty()) {
      DogLog.log("Trailblazer3/Diagnostic", result.diagnostic());
      lastLoggedDiagnostic = result.diagnostic();
    }
    if (result.infeasible() != lastLoggedInfeasible || result.infeasible()) {
      DogLog.log("Trailblazer3/Infeasible", result.infeasible());
      lastLoggedInfeasible = result.infeasible();
    }
  }

  void logCalibrationIdle(AutoSelection state, boolean transitionedFromAuto) {
    if (transitionedFromAuto || lastLoggedSelection != state) {
      DogLog.log("Autos/PreviewPath", EMPTY_PATH);
      DogLog.log("Autos/SelectedAutoName", state.toString());
      DogLog.log("Autos/Step", "CALIBRATION_IDLE");
      DogLog.log("Autos/AtExpectedStartPose", true);
      lastLoggedSelection = state;
      lastNearStart = true;
    }
  }

  void logIdle(
      AutoSelection state,
      TrailblazerAuto<?> auto,
      Pose2d actualPose,
      boolean transitionedFromAuto) {
    boolean nearStart = auto.isNearStartPose(actualPose);
    if (transitionedFromAuto
        || lastLoggedSelection != state
        || lastNearStart != nearStart
        || DiagnosticCadence.shouldLogRoutine()) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      DogLog.log("Autos/Step", "IDLE");
      DogLog.log("Autos/AtExpectedStartPose", nearStart);
      lastLoggedSelection = state;
      lastNearStart = nearStart;
    }
  }

  void publishPreview(TrailblazerAuto<?> auto, Pose2d fallbackPose) {
    DogLog.log("Autos/PreviewPath", auto.previewPath(fallbackPose));
  }
}
