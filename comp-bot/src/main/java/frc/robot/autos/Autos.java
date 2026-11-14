package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.autos.AutoChooser;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.config.RobotKind;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;

public class Autos extends StateMachineSubsystem<AutoSelection> {
  private static final String DIAGNOSTIC_KEY = "Trailblazer3/Diagnostic";

  private static Alliance alliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  private final AutoChooser<AutoSelection> autoChooser =
      new AutoChooser<>(AutoSelection.values(), AutoSelection.DO_NOTHING);
  private final RobotManager robotManager;
  private final Trailblazer trailblazer;
  private boolean started;

  private boolean rejected;
  private TrailblazerTrenchAuto activeAuto;
  private double previousMeasuredSpeed;
  private double previousMeasurementTime;
  private boolean braking;

  public Autos(RobotManager robotManager, Trailblazer trailblazer) {
    super(SubsystemPriority.AUTOS, AutoSelection.DO_NOTHING);
    this.robotManager = robotManager;
    this.trailblazer = trailblazer;
  }

  @Override
  protected AutoSelection getNextState(AutoSelection currentState) {
    return DriverStation.isEnabled() ? currentState : autoChooser.getSelectedAuto();
  }

  @Override
  protected void whileInState(AutoSelection state) {
    boolean trenchSelected = state == AutoSelection.TRAILBLAZER_3_RIGHT_TRENCH;
    if (!DriverStation.isAutonomousEnabled()) {
      if (started) {
        trailblazer.update(
            RobotStateSnapshot.measured(
                Utils.getCurrentTimeSeconds(),
                robotManager.localization.getPose(),
                new ChassisSpeeds(),
                false));
        started = false;
        rejected = false;
        activeAuto = null;
      }
      robotManager.swerve.clearTrajectoryCommand();
      if (DriverStation.isDisabled()
          && trenchSelected
          && RobotKind.IS_COMP_BOT
          && DSOptions.RESET_POSE_FOR_AUTO.getAsBoolean()
          && (RobotBase.isSimulation()
              || DriverStation.isAutonomous()
              || DriverStation.isFMSAttached())) {
        robotManager.localization.resetPose(TrailblazerTrenchAuto.start(alliance()));
      }
      return;
    }

    if (!trenchSelected) {
      if (!started) {
        started = true;
        robotManager.homeDeployInAutoRequest();
        robotManager.homeShooterHoodRequest();
      }
      return;
    }

    if (rejected) {
      return;
    }
    var drive = robotManager.swerve.drivetrain.getStateCopy();
    var pose = drive.Pose;
    boolean firstTick = !started;
    if (firstTick) {
      started = true;
      activeAuto = new TrailblazerTrenchAuto(trailblazer, alliance());
      if (!RobotKind.IS_COMP_BOT || !activeAuto.acceptsStart(pose)) {
        DogLog.log(DIAGNOSTIC_KEY, "Comp bot or right-trench start pose required");
        rejected = true;
        return;
      }
    }

    if (firstTick
        || trailblazer.lastResult().status()
            != com.team581.trailblazer3.model.TickResult.RuntimeStatus.FAULT) {
      activeAuto.periodic();
    }
    var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.Speeds, pose.getRotation());
    var result =
        trailblazer.update(RobotStateSnapshot.measured(drive.Timestamp, pose, speeds, true));
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
  }
}
