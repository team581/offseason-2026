package frc.robot.autos;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.team581.math.MathHelpers;
import com.team581.signals.Signals;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.profiling.DiagnosticCadence;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.swerve.DriveRequestSamples;
import frc.robot.swerve.Swerve;
import java.util.ArrayDeque;
import java.util.concurrent.CompletableFuture;

final class AutoTelemetry {
  private record MotionSample(long timestamp, double commandedSpeed, double measuredSpeed) {}

  private static final long ACCELERATION_MIN_WINDOW_MICROS = 60_000;
  private static final long ACCELERATION_MAX_WINDOW_MICROS = 120_000;
  private static final long MAX_MOTION_GAP_MICROS = 100_000;

  private static final String DRIVE = "Trailblazer3/DriveRequest/";

  /** Packed triples retain individual acquisition times, including repeated or invalid readings. */
  private static double[] signalValues(BaseStatusSignal[] signals) {
    double[] row = new double[signals.length * 3];
    for (int i = 0; i < signals.length; i++) {
      var signal = signals[i];
      row[3 * i] = signal.getStatus().isOK() ? signal.getValueAsDouble() : Double.NaN;
      row[3 * i + 1] = signal.getTimestamp().getTime();
      row[3 * i + 2] = signal.getStatus().value;
    }
    return row;
  }

  private final Trailblazer trailblazer;
  private final ArrayDeque<MotionSample> motionSamples = new ArrayDeque<>();
  private String lastMotionStep = "";
  private AutoSelection lastLoggedSelection = AutoSelection.DO_NOTHING;
  private boolean lastNearStart;
  private RuntimeStatus lastLoggedStatus = RuntimeStatus.IDLE;
  private long lastLoggedGeneration = -1;

  private String lastLoggedDiagnostic = "";
  private boolean lastLoggedInfeasible = false;
  private final Swerve swerve;
  private final BaseStatusSignal[][] drives = new BaseStatusSignal[4][];
  private final BaseStatusSignal[] imu;
  private boolean schemaPublished;
  private long run;
  private double lastWheelTime = Double.NaN;
  private double lastWheelSpeed;
  private double biasSum;
  private double biasSquares;
  private int biasSamples;
  private double lastBiasTime = Double.NaN;
  private long readRevision = -1;
  private CompletableFuture<double[]> readback;

  private double readbackRequestedAt;

  AutoTelemetry(Trailblazer trailblazer, Swerve swerve) {
    this.swerve = swerve;
    for (int i = 0; i < drives.length; i++) {
      var motor = swerve.drivetrain.getModule(i).getDriveMotor();
      drives[i] =
          new BaseStatusSignal[] {
            motor.getMotorVoltage(false),
            motor.getSupplyVoltage(false),
            motor.getStatorCurrent(false),
            motor.getSupplyCurrent(false),
            motor.getFault_StatorCurrLimit(false),
            motor.getFault_SupplyCurrLimit(false)
          };
      Signals.forDevice(motor).addSignals(drives[i]);
    }
    var pigeon = swerve.drivetrain.getPigeon2();
    imu =
        new BaseStatusSignal[] {
          pigeon.getAccelerationX(false),
          pigeon.getAccelerationY(false),
          pigeon.getAccelerationZ(false),
          pigeon.getGravityVectorX(false),
          pigeon.getGravityVectorY(false),
          pigeon.getGravityVectorZ(false),
          pigeon.getRoll(false),
          pigeon.getPitch(false)
        };
    Signals.forDevice(pigeon).addSignals(imu);
    this.trailblazer = trailblazer;
  }

  private void collectLimits() {
    if (readback != null && readback.isDone()) {
      DogLog.log(DRIVE + "ReadbackRequestedAt", readbackRequestedAt);
      if (readback.isCompletedExceptionally()) {
        DogLog.log(DRIVE + "ReadbackFailedAt", readbackRequestedAt);
      } else {
        DogLog.log(DRIVE + "CurrentLimits", readback.join());
      }
      readback = null;
    }
  }

  private void drainApplied() {
    var buffer = swerve.trajectorySamples();
    var batch = buffer.drain();
    if (batch.length > 0) DogLog.log(DRIVE + "Applied", batch);
    DogLog.log(DRIVE + "DroppedTotal", buffer.dropped());
  }

  private void logDrive(AutoSelection state, double axis, boolean staged, boolean firstTick) {
    drainApplied();
    if (!schemaPublished) {
      schemaPublished = true;
      DogLog.log(DRIVE + "Runtime", swerve.trajectorySamples().runtimeName());
      DogLog.log(DRIVE + "AppliedColumns", DriveRequestSamples.COLUMNS);
      DogLog.log(
          DRIVE + "DriveColumns",
          "motor_V,supply_V,stator_A,supply_A,stator_limit_fault,supply_limit_fault; each=value,phoenix_s,status");
      DogLog.log(
          DRIVE + "ImuColumns",
          "ax_g,ay_g,az_g,gravity_x_g,gravity_y_g,gravity_z_g,roll_deg,pitch_deg; each=value,phoenix_s,status");
      DogLog.log(
          DRIVE + "LimitColumns",
          "FL,FR,BL,BR; each=phoenix_s,status,stator_enabled,stator_A,supply_enabled,supply_A,lower_A,lower_s");
      DogLog.log(DRIVE + "Simulation", RobotBase.isSimulation());
    }
    DogLog.log(DRIVE + "PhoenixMinusFpgaSeconds", Utils.fpgaToCurrentTime(0));
    DogLog.log(DRIVE + "ObservedOwner", swerve.appliedTrajectoryOwner());
    DogLog.log(DRIVE + "ObservationTime", Utils.getCurrentTimeSeconds());
    DogLog.log(DRIVE + "BatteryVolts", RobotController.getBatteryVoltage());
    DogLog.log(DRIVE + "RequestedSupplyAmps", swerve.requestedDriveSupplyAmps());
    logLimits(firstTick);
    var drive = swerve.drivetrain.getStateCopy();
    DogLog.log(DRIVE + "OdometryTime", drive.Timestamp);
    DogLog.log(DRIVE + "ModuleTargets", drive.ModuleTargets);
    DogLog.log(DRIVE + "ModuleStates", drive.ModuleStates);
    DogLog.log(DRIVE + "Pose", drive.Pose);
    for (int i = 0; i < drives.length; i++)
      DogLog.log(DRIVE + "Drive" + i, signalValues(drives[i]));
    double[] raw = signalValues(imu);
    DogLog.log(DRIVE + "Imu", raw);
    if (state != AutoSelection.TRAILBLAZER_3_RIGHT_TRENCH) return;
    // Right Trench Leave travels along the alliance-adjusted start heading.
    var attitude =
        new Rotation3d(
            Math.toRadians(raw[18]),
            Math.toRadians(raw[21]),
            drive.Pose.getRotation().getRadians());
    var field =
        new Translation3d(raw[0] - raw[9], raw[3] - raw[12], raw[6] - raw[15]).rotateBy(attitude);
    double along = 9.80665 * (Math.cos(axis) * field.getX() + Math.sin(axis) * field.getY());
    double imuTime = raw[1];
    if (staged && Double.isFinite(along) && imuTime != lastBiasTime) {
      lastBiasTime = imuTime;
      biasSum += along;
      biasSquares += along * along;
      biasSamples++;
    }
    double bias = biasSamples > 0 ? biasSum / biasSamples : Double.NaN;
    DogLog.log(DRIVE + "AxisRadians", axis);
    DogLog.log(DRIVE + "PrelaunchBiasSamples", biasSamples);
    DogLog.log(DRIVE + "PrelaunchBiasMps2", bias);
    DogLog.log(
        DRIVE + "PrelaunchNoiseMps2",
        Math.sqrt(Math.max(0, biasSquares / biasSamples - bias * bias)));
    DogLog.log(DRIVE + "ImuAlongMps2", along - bias);
    var relative = drive.Pose.getRotation().getRadians() - axis;
    double speed =
        drive.Speeds.vxMetersPerSecond * Math.cos(relative)
            - drive.Speeds.vyMetersPerSecond * Math.sin(relative);
    DogLog.log(DRIVE + "WheelAlongMps", speed);
    if (drive.Timestamp > lastWheelTime) {
      DogLog.log(
          DRIVE + "WheelAlongMps2", (speed - lastWheelSpeed) / (drive.Timestamp - lastWheelTime));
    }
    lastWheelSpeed = speed;
    lastWheelTime = drive.Timestamp;
  }

  private void logLimits(boolean firstTick) {
    collectLimits();
    if (firstTick) readRevision = -1;
    if (readback != null || readRevision == swerve.driveLimitRevision()) return;
    readRevision = swerve.driveLimitRevision();
    readbackRequestedAt = Utils.getCurrentTimeSeconds();
    readback =
        CompletableFuture.supplyAsync(() -> {
          double[] row = new double[4 * 8];
          for (int i = 0; i < 4; i++) {
            var config = new CurrentLimitsConfigs();
            var status =
                swerve.drivetrain.getModule(i).getDriveMotor().getConfigurator().refresh(config);
            int j = i * 8;
            row[j] = Utils.getCurrentTimeSeconds();
            row[j + 1] = status.value;
            row[j + 2] = config.StatorCurrentLimitEnable ? 1 : 0;
            row[j + 3] = config.StatorCurrentLimit;
            row[j + 4] = config.SupplyCurrentLimitEnable ? 1 : 0;
            row[j + 5] = config.SupplyCurrentLimit;
            row[j + 6] = config.SupplyCurrentLowerLimit;
            row[j + 7] = config.SupplyCurrentLowerTime;
          }
          return row;
        });
  }

  private void logMotion(ChassisSpeeds command, ChassisSpeeds measured) {
    long timestamp = RobotController.getFPGATime();
    double commandedSpeed = MathHelpers.getLinearVelocity(command);
    double measuredSpeed = MathHelpers.getLinearVelocity(measured);
    if (!Double.isFinite(commandedSpeed)
        || !Double.isFinite(measuredSpeed)
        || !Double.isFinite(command.omegaRadiansPerSecond)
        || !Double.isFinite(measured.omegaRadiansPerSecond)) {
      motionSamples.clear();
      return;
    }

    DogLog.log("Trailblazer3/CommandedSpeed", commandedSpeed, "m/s");
    DogLog.log("Trailblazer3/MeasuredSpeed", measuredSpeed, "m/s");
    DogLog.log("Trailblazer3/CommandedYawRate", command.omegaRadiansPerSecond, "rad/s");
    DogLog.log("Trailblazer3/MeasuredYawRate", measured.omegaRadiansPerSecond, "rad/s");

    if (!motionSamples.isEmpty()) {
      long lastTimestamp = motionSamples.getLast().timestamp();
      if (timestamp <= lastTimestamp || timestamp - lastTimestamp > MAX_MOTION_GAP_MICROS) {
        motionSamples.clear();
      }
    }
    while (!motionSamples.isEmpty()
        && timestamp - motionSamples.getFirst().timestamp() > ACCELERATION_MAX_WINDOW_MICROS) {
      motionSamples.removeFirst();
    }

    // Use a trailing speed slope so replay and live telemetry can use the same samples.
    if (!motionSamples.isEmpty()
        && timestamp - motionSamples.getFirst().timestamp() >= ACCELERATION_MIN_WINDOW_MICROS) {
      var start = motionSamples.getFirst();
      double elapsed = (timestamp - start.timestamp()) * 1e-6;
      DogLog.log(
          "Trailblazer3/CommandedAcceleration",
          (commandedSpeed - start.commandedSpeed()) / elapsed,
          "m/s²");
      DogLog.log(
          "Trailblazer3/MeasuredAcceleration",
          (measuredSpeed - start.measuredSpeed()) / elapsed,
          "m/s²");
    }
    motionSamples.addLast(new MotionSample(timestamp, commandedSpeed, measuredSpeed));
  }

  void endRun() {
    drainApplied();
    lastWheelTime = Double.NaN;
    lastBiasTime = Double.NaN;
    biasSum = biasSquares = 0;
    biasSamples = 0;
    motionSamples.clear();
    lastMotionStep = "";
    lastLoggedStatus = RuntimeStatus.IDLE;
    lastLoggedDiagnostic = "";
    lastLoggedInfeasible = false;
  }

  void logActive(
      AutoSelection state,
      TrailblazerAuto<?> auto,
      TickResult result,
      ChassisSpeeds command,
      ChassisSpeeds measuredSpeeds,
      boolean firstTick,
      double plannerTime,
      boolean bumpOverride) {
    if (firstTick) run++;
    var stepData = result.velocityStep();
    double dt = stepData.durationSeconds();
    DogLog.log(DRIVE + "Run", run);
    DogLog.log(
        DRIVE + "PlannerOwner",
        bumpOverride ? "BUMP" : firstTick ? "FIRST_ENABLED" : dt > 0 ? "INTERPOLATED" : "DIRECT");
    DogLog.log(
        DRIVE + "PlannerColumns",
        "update_s,sequence,start_s,duration_s,start_vx,start_vy,start_omega,"
            + "end_vx,end_vy,end_omega,ax,ay,alpha (blue field SI)");
    DogLog.log(
        DRIVE + "Planner",
        new double[] {
          plannerTime,
          stepData.sequence(),
          stepData.startTime(),
          dt,
          stepData.startVx(),
          stepData.startVy(),
          stepData.startOmega(),
          result.commandedFieldSpeeds().vxMetersPerSecond,
          result.commandedFieldSpeeds().vyMetersPerSecond,
          result.commandedFieldSpeeds().omegaRadiansPerSecond,
          dt > 0 ? (result.commandedFieldSpeeds().vxMetersPerSecond - stepData.startVx()) / dt : 0,
          dt > 0 ? (result.commandedFieldSpeeds().vyMetersPerSecond - stepData.startVy()) / dt : 0,
          dt > 0
              ? (result.commandedFieldSpeeds().omegaRadiansPerSecond - stepData.startOmega()) / dt
              : 0
        });
    logDrive(state, auto.startPose().getRotation().getRadians(), false, firstTick);
    if (firstTick) {
      motionSamples.clear();
      lastLoggedStatus = RuntimeStatus.IDLE;
      lastLoggedDiagnostic = "";
      lastLoggedInfeasible = false;
    }
    if (firstTick || lastLoggedSelection != state) {
      DogLog.log("Autos/SelectedAutoName", state.toString());
      lastLoggedSelection = state;
    }

    var step = auto.currentStepName();
    if (!step.equals(lastMotionStep)) {
      motionSamples.clear();
      lastMotionStep = step;
    }
    DogLog.log("Autos/Step", step);
    DogLog.log("Trailblazer3/Status", result.status().name());
    if (result.status() == RuntimeStatus.RUNNING) {
      DogLog.log("Trailblazer3/CommandFieldSpeeds", command);
      DogLog.log("Trailblazer3/MeasuredFieldSpeeds", measuredSpeeds);
      logMotion(command, measuredSpeeds);
      DogLog.log("Trailblazer3/LimitingFactor", result.limitingFactor().name());
    } else {
      motionSamples.clear();
    }
    logPlannedPath();
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
    collectLimits();
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

  void logPlannedPath() {
    long generation = trailblazer.pathGeneration();
    if (generation != lastLoggedGeneration) {
      lastLoggedGeneration = generation;
      DogLog.log(
          "Trailblazer3/PlannedPath", trailblazer.sampledPathPoses(Trailblazer.PATH_LOG_SPACING));
    }
  }

  void logStaged(AutoSelection state, ChassisSpeeds launch, double axis) {
    if (state == AutoSelection.DO_NOTHING) return;
    DogLog.log(DRIVE + "Run", run + 1);
    DogLog.log(DRIVE + "PlannerOwner", "STAGED");
    DogLog.log(DRIVE + "StagedFieldSpeeds", launch);
    logDrive(state, axis, true, false);
  }
}
