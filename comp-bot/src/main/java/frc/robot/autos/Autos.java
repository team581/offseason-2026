package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.autos.AutoChooser;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.profiling.LoopTiming;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import java.util.EnumSet;
import java.util.Map;
import java.util.Optional;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private static final String DIAGNOSTIC_KEY = "Trailblazer3/Diagnostic";

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private final TrailblazerCalibrationAuto calibration;
  private boolean started;
  private boolean hasEnabledAuto;
  private final EnumSet<AutoSelection> warmedUpAutos = EnumSet.noneOf(AutoSelection.class);

  private TrailblazerAuto<?> activeAuto;
  private AutoSelection preparedSelection;
  private Alliance preparedAlliance;
  private double previousMeasuredSpeed;
  private double previousMeasurementTime;
  private boolean braking;
  private Alliance autoAlliance = Alliance.Blue;

  public Autos(RobotManager robotManager, Trailblazer trailblazer, Vision vision) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
    calibration = new TrailblazerCalibrationAuto(robotManager, vision);
  }

  private void prepareAuto(AutoSelection selection) {
    long activationStart = LoopTiming.start();
    activeAuto =
        selection == AutoSelection.TRAILBLAZER_3_TRENCH_ROTATION_TEST
            ? new TrailblazerTrenchRotationAuto(trailblazer, autoAlliance)
            : new TrailblazerTrenchAuto(trailblazer, autoAlliance);
    activeAuto.periodic();
    preparedSelection = selection;
    preparedAlliance = autoAlliance;
    LoopTiming.end("Scheduler/Autos/Activate", activationStart);
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    return DriverStation.isEnabled() ? currentState : autoChooser.getSelectedAuto();
  }

  @Override
  protected void whileInState(AutoSelection state) {
    boolean rotationSelected = state == AutoSelection.TRAILBLAZER_3_TRENCH_ROTATION_TEST;
    boolean trailblazerSelected =
        rotationSelected || state == AutoSelection.TRAILBLAZER_3_RIGHT_TRENCH;
    if (!DriverStation.isAutonomousEnabled()) {
      calibration.end();
      autoAlliance = DriverStation.getAlliance().orElse(autoAlliance);
      if (started) {
        trailblazer.update(
            RobotStateSnapshot.measured(
                Utils.getCurrentTimeSeconds(),
                robotManager.localization.getPose(),
                new ChassisSpeeds(),
                false));
        started = false;
        activeAuto = null;
      }
      robotManager.swerve.clearTrajectoryCommand();
      if (DriverStation.isDisabled()
          && trailblazerSelected
          && !hasEnabledAuto
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && (RobotBase.isSimulation()
              || DriverStation.isAutonomous()
              || DriverStation.isFMSAttached())) {
        robotManager.localization.resetPose(
            rotationSelected
                ? TrailblazerTrenchRotationAuto.start(autoAlliance)
                : TrailblazerTrenchAuto.start(autoAlliance));
      }
      if (DriverStation.isDisabled() && trailblazerSelected) {
        // Phoenix receives the launch direction while the Driver Station inhibits motor output.
        double direction = autoAlliance == Alliance.Red ? 1 : -1;
        robotManager.swerve.setTrajectoryCommand(
            rotationSelected
                ? new ChassisSpeeds(0, -direction, 0)
                : new ChassisSpeeds(direction, 0, 0));
        if (warmedUpAutos.add(state)) {
          long warmupStart = LoopTiming.start();
          if (rotationSelected) {
            TrailblazerTrenchRotationAuto.warmup(autoAlliance);
          } else {
            TrailblazerTrenchAuto.warmup(autoAlliance);
          }
          LoopTiming.end("Scheduler/Autos/Warmup", warmupStart);
        }
        if (activeAuto == null || preparedSelection != state || preparedAlliance != autoAlliance) {
          prepareAuto(state);
        }
      } else {
        activeAuto = null;
      }
      return;
    }

    hasEnabledAuto = true;
    var calibrationTest = TrailblazerCalibrationAuto.Test.fromSelection(state);
    if (calibrationTest != null) {
      calibration.run(calibrationTest);
      return;
    }

    if (!trailblazerSelected) {
      if (!started) {
        started = true;
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      return;
    }

    boolean firstTick = !started;
    if (firstTick) {
      if (activeAuto == null || preparedSelection != state || preparedAlliance != autoAlliance) {
        prepareAuto(state);
      }
      started = true;
    } else if (trailblazer.lastResult().status()
        != com.team581.trailblazer3.model.TickResult.RuntimeStatus.FAULT) {
      activeAuto.periodic();
    }

    long odometryStart = LoopTiming.start();
    // Sample after activation so segment setup cannot age the snapshot used on the first tick.
    var drive = robotManager.swerve.drivetrain.getStateCopy();
    var pose = drive.Pose;
    var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.Speeds, pose.getRotation());
    double now = Utils.getCurrentTimeSeconds();
    LoopTiming.end("Scheduler/Autos/ReadOdometry", odometryStart);
    DogLog.log("Trailblazer3/OdometryAgeMs", (now - drive.Timestamp) * 1000);
    var snapshot = RobotStateSnapshot.measured(drive.Timestamp, pose, speeds, true);
    long updateStart = LoopTiming.start();
    var result =
        trailblazer.update(new TickInput(now, snapshot, Map.of(), Map.of(), Optional.empty()));
    LoopTiming.end("Scheduler/Autos/UpdatePlanner", updateStart);
    long outputStart = LoopTiming.start();
    robotManager.swerve.setTrajectoryCommand(result.commandedFieldSpeeds());
    DogLog.log("Trailblazer3/Status", result.status().name());
    DogLog.log("Trailblazer3/Infeasible", result.infeasible());
    DogLog.log("Trailblazer3/ProfileLimit", result.profileLimitingFactor().name());
    DogLog.log("Trailblazer3/CommandLimit", result.commandLimitingFactor().name());
    DogLog.log("Trailblazer3/Progress", result.progress());
    double measuredSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    DogLog.log("Trailblazer3/MeasuredSpeedMps", measuredSpeed);
    if (!firstTick && drive.Timestamp > previousMeasurementTime) {
      DogLog.log(
          "Trailblazer3/MeasuredAccelerationMps2",
          (measuredSpeed - previousMeasuredSpeed) / (drive.Timestamp - previousMeasurementTime));
    }
    previousMeasuredSpeed = measuredSpeed;
    previousMeasurementTime = drive.Timestamp;
    if (firstTick) {
      braking = false;
    }
    var command = result.commandedFieldSpeeds();
    DogLog.log(
        "Trailblazer3/CommandSpeedMps",
        Math.hypot(command.vxMetersPerSecond, command.vyMetersPerSecond));
    trailblazer
        .motionTelemetry()
        .ifPresent(motion -> {
          DogLog.log("Trailblazer3/ProfileSpeedMps", motion.profileSpeed());
          if (!braking && motion.commandedAcceleration() < -1e-6) {
            braking = true;
            DogLog.log("Trailblazer3/BrakingStartMeters", result.progress());
          }
          DogLog.log("Trailblazer3/ProfileAccelerationMps2", motion.profileAcceleration());
          DogLog.log("Trailblazer3/CommandAccelerationMps2", motion.commandedAcceleration());
          DogLog.log("Trailblazer3/AvailableAccelerationMps2", motion.availableAcceleration());
          DogLog.log("Trailblazer3/MaxSpeedMps", motion.maxSpeed());
          DogLog.log("Trailblazer3/MaxBrakingMps2", motion.maxBraking());
          DogLog.log("Trailblazer3/MaxAngularSpeedRadPerSec", motion.maxAngularSpeed());
          DogLog.log(
              "Trailblazer3/MaxAngularAccelerationRadPerSec2", motion.maxAngularAcceleration());
          DogLog.log("Trailblazer3/DistanceRemainingMeters", motion.distanceRemaining());
          DogLog.log("Trailblazer3/PositionErrorMeters", motion.positionError());
          DogLog.log("Trailblazer3/StopDwellSeconds", motion.dwellSeconds());
        });
    DogLog.log(DIAGNOSTIC_KEY, result.diagnostic());
    LoopTiming.end("Scheduler/Autos/Output", outputStart);
  }
}
