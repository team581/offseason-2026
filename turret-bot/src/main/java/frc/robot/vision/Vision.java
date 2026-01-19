package frc.robot.vision;

import java.util.Optional;

import com.team581.math.MathHelpers;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.imu.Imu;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.OptionalTagResult;

public class Vision extends StateMachineSubsystem<VisionState> {
  private static final Transform2d TURRET_TO_CAMERA =
      new Transform2d(Units.inchesToMeters(-5.3), 0.0, Rotation2d.kZero);

  private static final Transform2d TURRET_TO_ROBOT =
      new Transform2d(Units.inchesToMeters(-0.5), 0.0, Rotation2d.kZero);

  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final TimeInterpolatableBuffer<Rotation2d> turretBuffer =
      TimeInterpolatableBuffer.createBuffer(2.0);

  private final Imu imu;
  private final Limelight turretLimelight;
  private final Limelight backLimelight;
  private final Limelight frontLimelight;

  private OptionalTagResult turretResult = new OptionalTagResult();
  private OptionalTagResult adjustedTurretResult = new OptionalTagResult();
  private OptionalTagResult backResult = new OptionalTagResult();
  private OptionalTagResult frontResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public Vision(
      Imu imu, Limelight turretLimelight, Limelight backLimelight, Limelight frontLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.turretLimelight = turretLimelight;
    this.backLimelight = backLimelight;
    this.frontLimelight = frontLimelight;
  }

  @Override
  protected void collectInputs() {
    robotAngularVelocity = imu.getRobotAngularVelocity();

    turretResult = turretLimelight.getTagResult();
    backResult = backLimelight.getTagResult();
    frontResult = frontLimelight.getTagResult();

    adjustedTurretResult = getAdjustedTurretLimelightTagResult(turretResult);

    if (turretResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }
    seeingTagDebounced = seeingTagDebouncer.calculate(seeingTag);
    if (DriverStation.isDisabled()) {
      seenTagRecentlyForReset = true;
    } else {
      seenTagRecentlyForReset = seeingTagForPoseResetDebouncer.calculate(seeingTag);
    }
  }

  // Call this in turret's periodic() or a fast telemetry thread
  public void addTurretObservation(double timestamp, Rotation2d angle, double turretAngularVelocity) {
    turretBuffer.addSample(timestamp, angle);
    turretLimelight.sendImuData(robotHeading, turretAngularVelocity+robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
  }

  private Optional<Rotation2d> getAngleAtTimestamp(double timestamp) {
    return turretBuffer.getSample(timestamp);
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getAdjustedTurretLimelighTagResult() {
    return adjustedTurretResult;
  }

  private OptionalTagResult getAdjustedTurretLimelightTagResult(OptionalTagResult turretResult) {
    if (turretResult.isEmpty()) {
      return adjustedTurretResult.empty();
    }

    var turretLimelightResult = turretResult.orElseThrow();

    var mT1Pose = turretLimelightResult.pose();
    var mT1Timestamp = turretLimelightResult.timestamp();
    var cameraToTurretTransform = TURRET_TO_CAMERA.inverse();
    var fieldToTurretPose = mT1Pose;

    if (FeatureFlags.VISION_CAMERA_POSITION_COMPENSATION.getAsBoolean()) {
      fieldToTurretPose = mT1Pose.plus(cameraToTurretTransform);
    }

    // Look up the turret angle at the specific image timestamp
    var robotToTurretObservation = getAngleAtTimestamp(mT1Timestamp);
    if (robotToTurretObservation.isEmpty()) {
      DogLog.logFault("Could not get turret angle at timestamp");
      return adjustedTurretResult.empty();
    }
    DogLog.log("Vision/TurretObservation", robotToTurretObservation.orElseThrow().getDegrees());
    DogLog.clearFault("Could not get turret angle at timestamp");

    // Create transform representing the rotation from Turret back to Robot
    // If the turret is at +90 degrees, we rotate -90 degrees to get back to the robot front.
    var turretToRobot =
        MathHelpers.transform2dFromRotation(robotToTurretObservation.orElseThrow().unaryMinus());

    var fieldToRobotEstimate = fieldToTurretPose;

    // Add this rotation to the Turret's Field Pose to finally get the Robot's Field Pose
    if (FeatureFlags.VISION_TURRET_ANGLE_COMPENSATION.getAsBoolean()) {
      fieldToRobotEstimate = fieldToTurretPose.plus(turretToRobot);
    }

    if (FeatureFlags.VISION_TURRET_POSITION_COMPENSATION.getAsBoolean()) {
      fieldToRobotEstimate = fieldToRobotEstimate.plus(TURRET_TO_ROBOT.inverse());
    }
    DogLog.log("Vision/AdjustedTurretPose", fieldToRobotEstimate);
    return adjustedTurretResult.update(
        fieldToRobotEstimate, mT1Timestamp, turretLimelightResult.standardDevs());
  }

  public OptionalTagResult getBackLimelightTagResult() {
    return backResult;
  }

  public OptionalTagResult getFrontLimelightTagResult() {
    return frontResult;
  }

  public boolean seeingTagDebounced() {
    return seeingTagDebounced;
  }

  public boolean seenTagRecentlyForReset() {
    return seenTagRecentlyForReset;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    if (state == VisionState.HUB_TAGS && !FeatureFlags.VISION_HUB_TAGS_FILTER.getAsBoolean()) {
      state = VisionState.TAGS;
    }
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        turretLimelight.setState(LimelightState.TAGS);
        backLimelight.setState(LimelightState.TAGS);
        frontLimelight.setState(LimelightState.TAGS);
      }
      case HUB_TAGS -> {
        turretLimelight.setState(LimelightState.HUB_TAGS);
        backLimelight.setState(LimelightState.HUB_TAGS);
        frontLimelight.setState(LimelightState.HUB_TAGS);
      }
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    // Send IMU data to all limelights
    // Set turret limelight angular velocity from turret
    backLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    frontLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);
  }

  public boolean isAnyCameraOffline() {
    return turretLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyCameraOnlineForTags() {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return turretLimelight.isOnlineForTags()
        || backLimelight.isOnlineForTags()
        || frontLimelight.isOnlineForTags();
  }
}
