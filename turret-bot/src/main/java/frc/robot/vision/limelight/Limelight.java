package frc.robot.vision.limelight;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.ReusableOptional;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.results.OptionalGamePieceResult;
import frc.robot.vision.results.OptionalTagResult;
import java.util.Locale;
import java.util.OptionalDouble;

public class Limelight extends StateMachineSubsystem<LimelightState> {
  private static final int[] VALID_APRILTAGS =
      new int[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  private static final double IS_OFFLINE_TIMEOUT = 3;

  private final String limelightTableName;
  private final String name;
  private final CameraConfig config;

  private final Timer limelightTimer = new Timer();
  private final Timer seedImuTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;

  private OptionalTagResult lastGoodTagResult = new OptionalTagResult();
  private OptionalTagResult tagResult = new OptionalTagResult();

  private OptionalGamePieceResult coralResult = new OptionalGamePieceResult();
  private OptionalGamePieceResult algaeResult = new OptionalGamePieceResult();

  private double angularVelocity = 0.0;
  private boolean updatedLimelightPos = false;

  public Limelight(String name, LimelightState initialState, CameraConfig config) {
    // TODO(jonahsnider): Make Limelight state logging work with multiple instances, not just
    // singleton
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.config = config;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, angularVelocity, pitch, pitchRate, roll, rollRate);
    this.angularVelocity = angularVelocity;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  public OptionalGamePieceResult getCoralResult() {
    return getState() == LimelightState.CORAL ? coralResult : coralResult.empty();
  }

  public OptionalGamePieceResult getAlgaeResult() {
    return getState() == LimelightState.ALGAE ? algaeResult : coralResult.empty();
  }

  public OptionalTagResult getTagResult() {
    if (getState() != LimelightState.TAGS) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    var mTEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
    var mTEstimateTimestamp = mTEstimate.timestampSeconds;

    if (mTEstimate == null) {
      return tagResult.empty();
    }

    if (Math.abs(angularVelocity) > 360) {
      return tagResult.empty();
    }
    if (mTEstimate.tagCount == 0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);

      return tagResult.empty();
    }
    if (mTEstimate.rawFiducials.length == 1) {
      double ambiguity = mTEstimate.rawFiducials[0].ambiguity;
      if (ambiguity >= 0.7) {
        DogLog.timestamp("Vision/" + name + "/Tags/AmbiguityFilter");
        return tagResult.empty();
      }
    }

    var mtPose = mTEstimate.pose;

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (mtPose.getX() == 0.0 && mtPose.getY() == 0.0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    var distance = mTEstimate.avgTagDist;
    var xyDev = 0.01 * Math.pow(distance, 1.2);
    var thetaDev = 0.03 * Math.pow(distance, 1.2);

    var devs = VecBuilder.fill(xyDev, xyDev, thetaDev);

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", mtPose);
    DogLog.log("Vision/" + name + "/Tags/MT2Timestamp", mTEstimateTimestamp);
    DogLog.log("Vision/" + name + "/Tags/DistanceFromTag", distance);
    return tagResult.update(mtPose, mTEstimateTimestamp, devs);
  }

  private OptionalGamePieceResult getRawCoralResult() {
    if (getState() != LimelightState.CORAL) {
      return coralResult.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return coralResult.empty();
    }
    var coralTx = t2d[4];
    var coralTy = t2d[5];
    if (coralTx == 0.0 || coralTy == 0.0) {
      return coralResult.empty();
    }

    DogLog.log("Vision/" + name + "/Coral/tx", coralTx);
    DogLog.log("Vision/" + name + "/Coral/ty", coralTy);

    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;

    return coralResult.update(coralTx, coralTy, timestamp);
  }

  public OptionalDouble handoffTx() {
    if (getState() != LimelightState.HANDOFF) {
      return OptionalDouble.empty();
    }

    var t2d = LimelightHelpers.getT2DArray(limelightTableName);

    if (t2d.length != 17) {
      return OptionalDouble.empty();
    }
    var tv = t2d[0];

    if (tv == 0) {
      return OptionalDouble.empty();
    }

    var tx = t2d[4];
    if (tx == 0.0) {
      return OptionalDouble.empty();
    }

    return OptionalDouble.of(tx);
  }

  private OptionalGamePieceResult getRawAlgaeResult() {
    if (getState() != LimelightState.ALGAE) {
      return algaeResult.empty();
    }
    var t2d = LimelightHelpers.getT2DArray(limelightTableName);
    if (t2d.length == 0) {
      return algaeResult.empty();
    }
    var algaeTx = t2d[4];
    var algaeTy = t2d[5];
    if (algaeTx == 0.0 || algaeTy == 0.0) {
      return algaeResult.empty();
    }

    DogLog.log("Vision/" + name + "/Algae/tx", algaeTx);
    DogLog.log("Vision/" + name + "/Algae/ty", algaeTy);

    var latency = t2d[2] + t2d[3];
    var latencySeconds = latency / 1000.0;
    var timestamp = Timer.getFPGATimestamp() - latencySeconds;

    return algaeResult.update(algaeTx, algaeTy, timestamp);
  }

  @Override
  protected void collectInputs() {
    tagResult = getTagResult();
    if (tagResult.isPresent()) {
      lastGoodTagResult = tagResult;
    }
    coralResult = getRawCoralResult();
    algaeResult = getRawAlgaeResult();
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    if (DriverStation.isDisabled()) {
      if (!updatedLimelightPos && getCameraHealth() != CameraHealth.OFFLINE) {
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightTableName,
            config.forward(),
            config.right(),
            config.up(),
            config.roll(),
            config.pitch(),
            config.yaw());

        updatedLimelightPos = true;
      }
      if (config.model() == LimelightModel.FOUR) {
        LimelightHelpers.setLimelightNTDouble(limelightTableName, "throttle_set", 5);
      }
    } else {
      LimelightHelpers.setLimelightNTDouble(limelightTableName, "throttle_set", 0);
    }
    DogLog.log("Vision/" + name + "/State", getState());

    var lastTagTimestamp =
        lastGoodTagResult.isPresent()
            ? lastGoodTagResult.orElseThrow().timestamp()
            : Double.MIN_VALUE;

    if (Timer.getTimestamp() - lastTagTimestamp > 30) {
      DogLog.logFault(
          limelightTableName + " has not seen a tag in the last 30 seconds", AlertType.kWarning);
    } else {
      DogLog.clearFault(limelightTableName + " has not seen a tag in the last 30 seconds");
    }

    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
    switch (getState()) {
      case TAGS -> {
        if (limelightTimer.hasElapsed(5.0)) {
          // LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
        }
        updateHealth(tagResult);
      }
      case CORAL -> updateHealth(coralResult);
      case ALGAE -> updateHealth(algaeResult);
      case HANDOFF -> updateHealth(coralResult);
    }

    // TODO: Remove once Limelights are upgraded
    LimelightHelpers.SetIMUMode(limelightTableName, 0);
    // if (limelightModel == LimelightModel.FOUR) {
    //   LimelightHelpers.SetIMUMode(limelightTableName, seedIMUTimer.hasElapsed(2.0) ? 4 : 3);
    // } else {
    //   // TODO: Can remove once we have upgraded all the Limelights
    //   LimelightHelpers.SetIMUMode(limelightTableName, 0);
    // }
  }

  @Override
  public void autonomousInit() {
    if (config.model() != LimelightModel.THREE) {
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
    }
    seedImuTimer.reset();
    seedImuTimer.start();
  }

  @Override
  public void teleopInit() {
    if (config.model() != LimelightModel.THREE) {
      LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
    }
  }

  private void updateHealth(ReusableOptional<?> result) {
    var newHeartbeat = LimelightHelpers.getLimelightNTDouble(limelightTableName, "hb");
    DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      DogLog.logFault(name.toUpperCase(Locale.US) + "LIMELIGHT IS OFFLINE", AlertType.kError);
      return;
    } else {
      DogLog.clearFault(name.toUpperCase(Locale.US) + "LIMELIGHT IS OFFLINE");
    }

    if (result.isPresent()) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  public void setBlinkEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    }
  }

  public CameraHealth getCameraHealth() {
    DogLog.log("Vision/" + name + "/Health", cameraHealth);
    return cameraHealth;
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, OFF -> getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
  }
}
