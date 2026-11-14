package frc.robot.autos;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.signals.Signals;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.deploy.DeployState;
import frc.robot.robot_manager.RobotManager;
import frc.robot.vision.Vision;

final class TrailblazerCalibrationAuto {
  private enum Phase {
    HOME_OUTWARD,
    RETRACT,
    ACQUIRE_HUB,
    ALIGN,
    TEST,
    SETTLE,
    RETURN,
    DONE,
    ABORTED
  }

  enum Test {
    LAG(0.55, 0.85, 1.10),
    TRACTION(3.0, 5.0, 9.0),
    BRAKING(1.6, 3.2, 8.0);

    static Test fromSelection(AutoSelection selection) {
      return switch (selection) {
        case TRAILBLAZER_3_CALIBRATE_LAG -> LAG;
        case TRAILBLAZER_3_CALIBRATE_TRACTION -> TRACTION;
        case TRAILBLAZER_3_CALIBRATE_BRAKING -> BRAKING;
        default -> null;
      };
    }

    private final double[] levels;

    Test(double... levels) {
      this.levels = levels;
    }

    double duration(int pass) {
      return switch (this) {
        case LAG -> 0.65;
        case TRACTION -> 0.75;
        case BRAKING -> 0.95 + 1.35 / levels[pass];
      };
    }

    double speed(double elapsed, int pass) {
      double level = levels[pass];
      return switch (this) {
        case LAG -> elapsed < 0.65 ? level : 0;
        case TRACTION -> elapsed < 0.75 ? Math.min(1.65, level * elapsed) : 0;
        case BRAKING -> {
          double target = 1.35;
          double brakeStart = 0.95;
          yield elapsed < brakeStart
              ? Math.min(target, 2 * elapsed)
              : Math.max(0, target - level * (elapsed - brakeStart));
        }
      };
    }
  }

  private static final String KEY = "Trailblazer3/Calibration/";
  private static final int MODULE_COUNT = 4;
  private static final double STANDARD_GRAVITY = 9.80665;
  private static final double HUB_FRAME_MAX_AGE = 0.25;
  private static final double STOP_SPEED = 0.10;
  private static final double RETURN_TOLERANCE = 0.08;
  private static final double HEADING_TOLERANCE = Math.toRadians(4);

  private static void logSignal(String name, BaseStatusSignal signal, double value) {
    DogLog.log(KEY + name, signal.getStatus().isOK() ? value : Double.NaN);
    DogLog.log(KEY + name + "/Time", signal.getTimestamp().getTime());
  }

  private final RobotManager robot;
  private final Vision vision;
  private final BaseStatusSignal[] statorCurrent = new BaseStatusSignal[MODULE_COUNT];
  private final BaseStatusSignal[] supplyCurrent = new BaseStatusSignal[MODULE_COUNT];

  private final BaseStatusSignal[] imuSignals;
  private Phase phase;
  private double startTime;
  private double phaseStart;
  private double lastHubTimestamp;
  private boolean acquiredHub;
  private double stationarySince;
  private int pass;
  private Pose2d resetPose;

  private boolean started;

  TrailblazerCalibrationAuto(RobotManager robot, Vision vision) {
    this.robot = robot;
    this.vision = vision;
    var drivetrain = robot.swerve.drivetrain;
    for (int i = 0; i < MODULE_COUNT; i++) {
      var motor = drivetrain.getModule(i).getDriveMotor();
      statorCurrent[i] = motor.getStatorCurrent(false);
      supplyCurrent[i] = motor.getSupplyCurrent(false);
      Signals.forDevice(motor).addSignals(statorCurrent[i], supplyCurrent[i]);
    }
    var pigeon = drivetrain.getPigeon2();
    imuSignals =
        new BaseStatusSignal[] {
          pigeon.getAccelerationX(false),
          pigeon.getAccelerationY(false),
          pigeon.getAccelerationZ(false),
          pigeon.getPitch(false),
          pigeon.getRoll(false)
        };
    Signals.forDevice(pigeon).addSignals(imuSignals);
  }

  private void abort(String reason, double now) {
    robot.hopperManager.deploy.cancelOutwardHomingRequest();
    transition(Phase.ABORTED, now);
    DogLog.log(KEY + "AbortReason", reason);
    DogLog.log(KEY + "Phase", phase.name());
  }

  private void checkRun(SwerveDriveState drive, double now) {
    if (now - startTime >= 14.5) {
      abort(acquiredHub ? "Calibration time limit" : "No fresh alliance hub tag pose", now);
    } else if (phase != Phase.HOME_OUTWARD
        && phase != Phase.RETRACT
        && (now - drive.Timestamp > 0.10
            || !Double.isFinite(drive.Timestamp)
            || !Double.isFinite(drive.Pose.getX())
            || !Double.isFinite(drive.Pose.getY())
            || !Double.isFinite(drive.Pose.getRotation().getRadians()))) {
      abort("Invalid or stale drivetrain odometry", now);
    } else if (resetPose != null) {
      var offset = drive.Pose.relativeTo(resetPose);
      if (robot.hopperManager.deploy.getState() != DeployState.STOW
          || !robot.hopperManager.deploy.atGoal(DeployState.STOW)) {
        abort("Deploy left the retracted position", now);
      } else if (offset.getX() < -0.35
          || offset.getX() > 2.0
          || Math.abs(offset.getY()) > 0.45
          || Math.abs(offset.getRotation().getDegrees()) > 15) {
        abort("Robot left the calibration lane", now);
      }
    }
  }

  private void initialize(double now) {
    started = true;
    startTime = now;
    lastHubTimestamp = Double.NEGATIVE_INFINITY;
    acquiredHub = false;
    resetPose = null;
    pass = 0;
    DogLog.log(KEY + "AbortReason", "");
    robot.swerve.setLogCalibrationCommands(true);
    if (DriverStation.isFMSAttached()) {
      abort("Calibration is disabled on FMS", now);
      return;
    }
    robot.idleRequest();
    robot.hopperManager.setDriverWantsIntake(false);
    robot.hopperManager.setDriverWantsEject(false);
    robot.hopperManager.setCalibrationWantsStow(true);
    robot.hopperManager.idleRequest();
    robot.hopperManager.deploy.homeOutwardRequest();
    transition(Phase.HOME_OUTWARD, now);
  }

  private void log(Test test, SwerveDriveState drive, double now, boolean hubVisible) {
    DogLog.log(KEY + "Test", test.name());
    DogLog.log(KEY + "Phase", phase.name());
    DogLog.log(KEY + "Pass", pass + 1);
    DogLog.log(KEY + "TestLevel", test.levels[pass]);
    DogLog.log(KEY + "SampleTime", now);
    DogLog.log(KEY + "OdometryTime", drive.Timestamp);
    DogLog.log(KEY + "Pose", drive.Pose);
    DogLog.log(KEY + "MeasuredRobotSpeeds", drive.Speeds);
    DogLog.log(KEY + "ModuleStates", drive.ModuleStates);
    DogLog.log(KEY + "ModuleTargets", drive.ModuleTargets);
    DogLog.log(KEY + "HubVisible", hubVisible);
    DogLog.log(KEY + "HubAcquired", acquiredHub);
    DogLog.log(KEY + "Simulation", RobotBase.isSimulation());
    DogLog.log(KEY + "BatteryVolts", RobotController.getBatteryVoltage());
    DogLog.log(KEY + "DeployState", robot.hopperManager.deploy.getState().name());

    for (int i = 0; i < MODULE_COUNT; i++) {
      logSignal(
          "Drive/" + i + "/StatorAmps", statorCurrent[i], statorCurrent[i].getValueAsDouble());
      logSignal(
          "Drive/" + i + "/SupplyAmps", supplyCurrent[i], supplyCurrent[i].getValueAsDouble());
    }
    logSignal("Imu/XMps2", imuSignals[0], imuSignals[0].getValueAsDouble() * STANDARD_GRAVITY);
    logSignal("Imu/YMps2", imuSignals[1], imuSignals[1].getValueAsDouble() * STANDARD_GRAVITY);
    logSignal("Imu/ZMps2", imuSignals[2], imuSignals[2].getValueAsDouble() * STANDARD_GRAVITY);
    logSignal("Imu/PitchDeg", imuSignals[3], imuSignals[3].getValueAsDouble());
    logSignal("Imu/RollDeg", imuSignals[4], imuSignals[4].getValueAsDouble());
  }

  private boolean stoppedFor(SwerveDriveState drive, double now, double duration) {
    if (MathHelpers.getLinearVelocity(drive.Speeds) > STOP_SPEED
        || Math.abs(drive.Speeds.omegaRadiansPerSecond) > 0.15) {
      stationarySince = Double.NaN;
    } else if (Double.isNaN(stationarySince)) {
      stationarySince = now;
    }
    return now - stationarySince >= duration;
  }

  private void transition(Phase next, double now) {
    phase = next;
    phaseStart = now;
    stationarySince = Double.NaN;
  }

  void end() {
    if (!started) {
      return;
    }
    if (phase != Phase.DONE && phase != Phase.ABORTED) {
      abort("Autonomous ended", Utils.getCurrentTimeSeconds());
    }
    robot.hopperManager.deploy.cancelOutwardHomingRequest();
    robot.hopperManager.setCalibrationWantsStow(false);
    robot.swerve.setLogCalibrationCommands(false);
    started = false;
  }

  void run(Test test) {
    var drive = robot.swerve.drivetrain.getStateCopy();
    double now = Utils.getCurrentTimeSeconds();
    if (!started) {
      initialize(now);
    }

    if (phase != Phase.DONE && phase != Phase.ABORTED) {
      vision.hubTagsRequest();
    }
    double fpgaNow = Timer.getFPGATimestamp();
    vision.forEachHubTagResult((camera, result) -> {
      lastHubTimestamp = Math.max(lastHubTimestamp, result.timestamp());
      DogLog.log(KEY + "Vision/" + camera + "/Pose", result.pose());
      DogLog.log(KEY + "Vision/" + camera + "/FpgaTimestamp", result.timestamp());
    });
    double hubFrameAge = fpgaNow - lastHubTimestamp;
    boolean hubVisible = hubFrameAge >= 0 && hubFrameAge <= HUB_FRAME_MAX_AGE;
    // Keep the first valid sighting for startup. Later gaps are recorded for log analysis.
    acquiredHub |= hubVisible || RobotBase.isSimulation();
    if (phase != Phase.DONE && phase != Phase.ABORTED) {
      checkRun(drive, now);
    }

    // Keep commands in the same zero-deadband, velocity-only path used by Trailblazer.
    var command = new ChassisSpeeds();
    boolean align = false;
    double elapsed = now - phaseStart;
    var deploy = robot.hopperManager.deploy;
    switch (phase) {
      case HOME_OUTWARD -> {
        if (deploy.getState() == DeployState.INTAKE) {
          deploy.stowRequest();
          transition(Phase.RETRACT, now);
        } else if (elapsed >= 2.0) {
          abort("Outward deploy homing timed out", now);
        }
      }
      case RETRACT -> {
        if (deploy.atGoal(DeployState.STOW)) {
          transition(Phase.ACQUIRE_HUB, now);
        } else if (elapsed >= 2.0) {
          abort("Deploy did not retract", now);
        }
      }
      case ACQUIRE_HUB -> {
        if (acquiredHub && stoppedFor(drive, now, 0.15)) {
          resetPose = drive.Pose;
          DogLog.log(KEY + "ResetPose", resetPose);
          transition(Phase.ALIGN, now);
        }
      }
      case ALIGN -> {
        var driveDirection = resetPose.getRotation().minus(drive.Pose.getRotation());
        align = true;
        robot.swerve.alignTrajectoryModules(driveDirection);
        boolean modulesAligned = true;
        for (var module : drive.ModuleStates) {
          modulesAligned &= Math.abs(module.angle.minus(driveDirection).getSin()) < 0.09;
        }
        if (elapsed >= 0.15 && modulesAligned && stoppedFor(drive, now, 0.10)) {
          transition(Phase.TEST, now);
        } else if (elapsed >= 0.8) {
          abort("Modules did not align", now);
        }
      }
      case TEST -> {
        if (elapsed >= test.duration(pass)) {
          transition(Phase.SETTLE, now);
        } else {
          double speed = test.speed(elapsed, pass);
          var heading = resetPose.getRotation();
          command = new ChassisSpeeds(speed * heading.getCos(), speed * heading.getSin(), 0);
        }
      }
      case SETTLE -> {
        if (stoppedFor(drive, now, 0.18)) {
          transition(Phase.RETURN, now);
        } else if (elapsed >= 1.3) {
          abort("Robot did not settle after pass", now);
        }
      }
      case RETURN -> {
        var error = resetPose.getTranslation().minus(drive.Pose.getTranslation());
        double distance = error.getNorm();
        double angle = resetPose.getRotation().minus(drive.Pose.getRotation()).getRadians();
        if (elapsed >= 3.2) {
          abort("Return to start timed out", now);
        } else if (distance <= RETURN_TOLERANCE && Math.abs(angle) <= HEADING_TOLERANCE) {
          if (stoppedFor(drive, now, 0.15)) {
            DogLog.log(KEY + "ReturnedPose", drive.Pose);
            if (pass + 1 == test.levels.length) {
              transition(Phase.DONE, now);
            } else {
              pass++;
              transition(Phase.ALIGN, now);
            }
          }
        } else {
          stationarySince = Double.NaN;
          var velocity = error.times(Math.min(2.5, 1.1 / Math.max(distance, 0.01)));
          command =
              new ChassisSpeeds(
                  velocity.getX(), velocity.getY(), MathUtil.clamp(3 * angle, -0.8, 0.8));
        }
      }
      case DONE, ABORTED -> {}
    }
    if (!align || phase == Phase.ABORTED) {
      robot.swerve.setTrajectoryCommand(command);
    }
    log(test, drive, now, hubVisible);
  }
}
