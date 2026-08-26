package frc.robot.vision.limelight;

import com.google.common.collect.ImmutableSet;
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.FmsUtil;
import com.team581.util.ReusableOptional;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.limelight.LimelightHelpers;
import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import com.team581.vision.limelight.LimelightHelpers.RawFiducial;
import com.team581.vision.limelight.PoseEstimateValidator;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Locale;
import java.util.OptionalDouble;
import java.util.Set;

public class Limelight extends StateMachineSubsystem<LimelightState> {
  private static final double USE_MT1_ROTATION_THRESHOLD_INCHES = 40;

  private static final int[] VALID_APRILTAGS =
      new int[] {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
        26, 27, 28, 29, 30, 31, 32
      };

  private static final Set<Integer> RED_HUB_TAGS_SET = ImmutableSet.of(2, 3, 4, 5, 8, 9, 10, 11);
  private static final Set<Integer> BLUE_HUB_TAGS_SET =
      ImmutableSet.of(18, 19, 20, 21, 24, 25, 26, 27);
  private static final int[] RED_HUB_TAGS =
      RED_HUB_TAGS_SET.stream().mapToInt(Integer::intValue).toArray();
  private static final int[] BLUE_HUB_TAGS =
      BLUE_HUB_TAGS_SET.stream().mapToInt(Integer::intValue).toArray();
  private static final double IS_OFFLINE_TIMEOUT = 3;

  private static final DoubleSubscriber ROBOT_VELOCITY_STD_DEV_MULTIPLIER =
      DogLog.tunable("Vision/RobotVelocityStdDevMultiplier", 0.0);

  public final String limelightTableName;

  public final CameraConfig config;
  private final String name;
  private final PoseEstimateValidator poseEstimateValidator;
  private final Timer limelightTimer = new Timer();

  private final Timer seedImuTimer = new Timer();
  private CameraHealth cameraHealth = CameraHealth.NO_TARGETS;
  private double limelightHeartbeat = -1;
  private double lastGoodTagTimestamp = Double.MIN_VALUE;

  private OptionalTagResult tagResult = new OptionalTagResult();
  private double angularVelocity = 0.0;

  private double linearVelocity = 0.0;
  private double robotHeading = 0.0;
  private boolean updatedLimelightPos = false;
  private PoseEstimate latestEstimate = new PoseEstimate();

  // Tracks whether latestEstimate was validated this loop, so seeingHubTag()
  // can reuse the result without re-running the validator (which would mutate
  // its duplicate-detection state).
  private boolean latestEstimateTrusted = false;
  // Reused every loop to avoid allocating a new Matrix/Vector for stddevs.
  private final Vector<N3> reusableStdDevs = new Vector<>(N3.instance);

  public Limelight(String name, LimelightState initialState, CameraConfig config) {
    super(SubsystemPriority.VISION, initialState);
    limelightTableName = "limelight-" + name;
    this.name = name;
    limelightTimer.start();
    this.config = config;
    this.poseEstimateValidator = new PoseEstimateValidator(name);
  }

  @Override
  public void autonomousInit() {
    seedImuTimer.reset();
    seedImuTimer.start();
  }

  @Override
  public void disabledInit() {
    if (config.model() == LimelightModel.FOUR) {
      LimelightHelpers.triggerRewindCapture(limelightTableName, 165.0);
    }
  }

  public CameraHealth getCameraHealth() {
    return cameraHealth;
  }

  public OptionalDouble getLimelightRotation() {
    if (RobotBase.isSimulation()) {
      return OptionalDouble.of(90 - (Math.random() * 5));
    }
    var maybeResult = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
    if (poseEstimateValidator.shouldTrust(maybeResult, angularVelocity, robotHeading)) {
      return OptionalDouble.of(maybeResult.pose.getRotation().getDegrees());
    }
    return OptionalDouble.empty();
  }

  public OptionalTagResult getTagResult() {
    return tagResult;
  }

  public boolean isOnlineForTags() {
    return switch (getState()) {
      case TAGS, HUB_TAGS, OFF -> getCameraHealth() != CameraHealth.OFFLINE;
      default -> false;
    };
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
        LimelightHelpers.SetThrottle(limelightTableName, 10);
      }
    } else {
      LimelightHelpers.SetThrottle(limelightTableName, 0);
    }
    DogLog.log("Vision/" + name + "/State", getState());

    if (getState() == LimelightState.TAGS || getState() == LimelightState.HUB_TAGS) {
      if (Timer.getTimestamp() - lastGoodTagTimestamp > 30) {
        DogLog.logFault(
            limelightTableName + " has not seen a tag in the last 30 seconds", AlertType.kWarning);
      } else {
        DogLog.clearFault(limelightTableName + " has not seen a tag in the last 30 seconds");
      }
    } else {
      DogLog.clearFault(limelightTableName + " has not seen a tag in the last 30 seconds");
    }

    LimelightHelpers.setPipelineIndex(limelightTableName, getState().pipelineIndex);
    switch (getState()) {
      case TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, VALID_APRILTAGS);
        updateHealth(tagResult);
      }
      case HUB_TAGS -> {
        LimelightHelpers.SetFiducialIDFiltersOverride(limelightTableName, getActiveHubTags());
        updateHealth(tagResult);
      }
      case CLUSTER_MAP -> {
        updateHealth(LimelightHelpers.getTV(limelightTableName));
      }
      case OFF -> {}
    }

    DogLog.log("Vision/" + name + "/Health", cameraHealth);

    LimelightHelpers.SetIMUMode(limelightTableName, 0);
  }

  public boolean seeingHubTag() {
    if (!latestEstimateTrusted) {
      return false;
    }

    for (RawFiducial fiducial : latestEstimate.rawFiducials) {
      if (getActiveHubTagsSet().contains(fiducial.id)) {
        return true;
      }
    }

    return false;
  }

  public void sendImuData(
      double robotHeading,
      double angularVelocity,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    LimelightHelpers.SetRobotOrientation(
        limelightTableName, robotHeading, 0.0, pitch, pitchRate, roll, rollRate);
    this.angularVelocity = angularVelocity;
    this.robotHeading = robotHeading;
  }

  public void setBlinkEnabled(boolean enabled) {
    if (enabled) {
      LimelightHelpers.setLEDMode_ForceBlink(limelightTableName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelightTableName);
    }
  }

  public void setRobotVelocity(double velocity) {
    this.linearVelocity = velocity;
  }

  public void setState(LimelightState state) {
    setStateFromRequest(state);
  }

  private OptionalTagResult computeTagResult() {
    latestEstimateTrusted = false;

    if (getState() != LimelightState.TAGS && getState() != LimelightState.HUB_TAGS) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    PoseEstimate mT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);
    latestEstimate = mT1Estimate;

    if (!poseEstimateValidator.shouldTrust(mT1Estimate, angularVelocity, robotHeading)) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return tagResult.empty();
    }

    var mTEstimateTimestamp = mT1Estimate.timestampSeconds;
    var mTPose = mT1Estimate.pose;
    var distance = mT1Estimate.avgTagDist;

    double linearVelocityAddition =
        linearVelocity > 2.0
            ? linearVelocity * ROBOT_VELOCITY_STD_DEV_MULTIPLIER.getAsDouble()
            : 0.0;

    var xyDev = 0.01 * Math.pow(distance, 0.8) + linearVelocityAddition;
    var thetaDev = 999.0;

    if (config.useMt2()) {
      PoseEstimate mT2Estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightTableName);

      if (!poseEstimateValidator.shouldTrust(mT2Estimate, angularVelocity, robotHeading)) {
        DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
        return tagResult.empty();
      }

      mTEstimateTimestamp = mT2Estimate.timestampSeconds;
      mTPose = mT2Estimate.pose;
      if (config.useMt1RotationCloseUp()
          && distance < Units.inchesToMeters(USE_MT1_ROTATION_THRESHOLD_INCHES)) {
        mTPose = new Pose2d(mTPose.getTranslation(), mT1Estimate.pose.getRotation());
        thetaDev = 0.03 * Math.pow(distance, 1.2);
      }
    }

    latestEstimateTrusted = true;

    reusableStdDevs.set(0, 0, xyDev);
    reusableStdDevs.set(1, 0, xyDev);
    reusableStdDevs.set(2, 0, thetaDev);

    DogLog.log(
        "Vision/" + name + "/Tags/TimestampDifference",
        Timer.getFPGATimestamp() - mTEstimateTimestamp);

    DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", mTPose);
    DogLog.log("Vision/" + name + "/Tags/MTTimestamp", mTEstimateTimestamp);
    return tagResult.update(mTPose, mTEstimateTimestamp, reusableStdDevs);
  }

  private int[] getActiveHubTags() {
    return FmsUtil.isRedAlliance() ? RED_HUB_TAGS : BLUE_HUB_TAGS;
  }

  private Set<Integer> getActiveHubTagsSet() {
    return FmsUtil.isRedAlliance() ? RED_HUB_TAGS_SET : BLUE_HUB_TAGS_SET;
  }

  private void updateHealth(boolean hasTargets) {
    var newHeartbeat = LimelightHelpers.getHeartbeat(limelightTableName);
    DogLog.log("Vision/" + name + "/Heartbeat", newHeartbeat);
    if (limelightHeartbeat != newHeartbeat) {
      limelightTimer.restart();
    }
    limelightHeartbeat = newHeartbeat;

    if (limelightTimer.hasElapsed(IS_OFFLINE_TIMEOUT) && RobotBase.isReal()) {
      cameraHealth = CameraHealth.OFFLINE;
      DogLog.logFault(name.toUpperCase(Locale.US) + " LIMELIGHT IS OFFLINE", AlertType.kError);
      return;
    } else {
      DogLog.clearFault(name.toUpperCase(Locale.US) + " LIMELIGHT IS OFFLINE");
    }

    if (hasTargets) {
      cameraHealth = CameraHealth.GOOD;
      return;
    }
    cameraHealth = CameraHealth.NO_TARGETS;
  }

  private void updateHealth(ReusableOptional<?> result) {
    updateHealth(result.isPresent());
  }

  @Override
  protected void collectInputs() {
    tagResult = computeTagResult();
    if (tagResult.isPresent()) {
      lastGoodTagTimestamp = tagResult.orElseThrow().timestamp();
    }
  }
}
