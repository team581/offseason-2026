package frc.robot.autos;

import com.team581.math.MathHelpers;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.profiling.DiagnosticCadence;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

final class AutoTelemetry {
  private static final Pose2d[] EMPTY_PATH = new Pose2d[0];

  private final Trailblazer trailblazer;
  private double previousMeasuredSpeed;
  private double previousMeasurementTime;
  private boolean hasPreviousMeasuredSpeed;
  private boolean braking;

  private @Nullable AutoSelection lastLoggedSelection;
  private boolean lastNearStart;
  private @Nullable RuntimeStatus lastLoggedStatus;
  private long lastLoggedGeneration = -1;

  AutoTelemetry(Trailblazer trailblazer) {
    this.trailblazer = trailblazer;
  }

  private void logPlanner(
      TrailblazerAuto auto, TickResult result, RobotStateSnapshot snapshot, boolean forceFullLog) {
    boolean logRoutine = forceFullLog || DiagnosticCadence.shouldLogRoutine();
    DogLog.log("Autos/Step", auto.currentStepName());
    DogLog.log("Autos/StepIndex", auto.currentStepIndex());
    DogLog.log("Autos/Finished", auto.isFinished());

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
    String diagnostic =
        switch (result.status()) {
          case CANCELLED, DISABLED -> "";
          default -> result.diagnostic();
        };
    DogLog.log("Trailblazer3/Diagnostic", diagnostic);
  }

  private void publishIdle(
      AutoSelection state,
      String step,
      Optional<Pose2d> expectedStart,
      Pose2d actualPose,
      boolean nearStart) {
    DogLog.log("Autos/SelectedAutoName", state.toString());
    DogLog.log("Autos/Step", step);
    DogLog.log("Autos/StepIndex", -1);
    DogLog.log("Autos/Finished", false);
    DogLog.log("Autos/HasStartPose", expectedStart.isPresent());
    DogLog.log("Autos/ExpectedStartPose", expectedStart.orElse(Pose2d.kZero));
    DogLog.log("Autos/AtExpectedStartPose", nearStart);
    DogLog.log("Autos/StartPoseWarning", expectedStart.isPresent() && !nearStart);
    var expected = expectedStart.orElse(actualPose);
    DogLog.log(
        "Autos/StartPoseDistanceMeters",
        actualPose.getTranslation().getDistance(expected.getTranslation()));
    DogLog.log(
        "Autos/StartPoseHeadingErrorDeg",
        Math.abs(actualPose.getRotation().minus(expected.getRotation()).getDegrees()));
    lastLoggedSelection = state;
    lastNearStart = nearStart;
    lastLoggedStatus = null;
  }

  void endRun() {
    lastLoggedGeneration = -1;
    DogLog.log("Trailblazer3/PlannedPath", EMPTY_PATH);
  }

  void logActive(
      AutoSelection state,
      TrailblazerAuto auto,
      TickResult result,
      RobotStateSnapshot snapshot,
      double now,
      boolean firstTick) {
    if (firstTick) {
      hasPreviousMeasuredSpeed = false;
      braking = false;
      lastLoggedStatus = null;
      lastLoggedGeneration = -1;
    }
    if (firstTick || lastLoggedSelection != state || DiagnosticCadence.shouldLogRoutine()) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      lastLoggedSelection = state;
    }

    DogLog.log(
        "Trailblazer3/OdometryAgeMs",
        snapshot.localizationValid()
            ? (now - snapshot.poseTimestampSeconds()) * 1000.0
            : Double.NaN);
    DogLog.log("Trailblazer3/OdometryReady", snapshot.localizationValid());
    boolean forceFullLog =
        firstTick
            || result.status() != lastLoggedStatus
            || result.infeasible()
            || result.status() != RuntimeStatus.RUNNING;
    lastLoggedStatus = result.status();

    logPlanner(auto, result, snapshot, forceFullLog);
  }

  void logCalibrationIdle(AutoSelection state, boolean transitionedFromAuto) {
    if (transitionedFromAuto
        || lastLoggedSelection != state
        || DiagnosticCadence.shouldLogRoutine()) {
      if (lastLoggedSelection != state) {
        DogLog.log("Autos/PreviewPath", EMPTY_PATH);
      }
      publishIdle(state, "CALIBRATION_IDLE", Optional.empty(), Pose2d.kZero, true);
    }
  }

  void logIdle(
      AutoSelection state, TrailblazerAuto auto, Pose2d actualPose, boolean transitionedFromAuto) {
    boolean nearStart = auto.isNearStartPose(actualPose);
    if (transitionedFromAuto
        || lastLoggedSelection != state
        || lastNearStart != nearStart
        || DiagnosticCadence.shouldLogRoutine()) {
      publishIdle(state, "IDLE", auto.startPose(), actualPose, nearStart);
    }
  }

  void publishPreview(TrailblazerAuto auto, Pose2d fallbackPose) {
    DogLog.log("Autos/PreviewPath", auto.previewPath(fallbackPose));
  }
}
