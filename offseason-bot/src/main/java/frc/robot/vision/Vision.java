package frc.robot.vision;

import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.Optional;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingHubTagDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Imu imu;
  private final Limelight turretLimelight;
  private final Limelight leftLimelight;
  private final Limelight rightLimelight;

  private final Limelight groundLimelight;

  private final TimeInterpolatableBuffer<Double> turretBuffer;

  private OptionalTagResult shooterResult = new OptionalTagResult();
  private OptionalTagResult leftResult = new OptionalTagResult();
  private OptionalTagResult rightResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;

  private boolean seeingHubTags = false;

  public Vision(
      Imu imu,
      Limelight turretLimelight,
      Limelight leftLimelight,
      Limelight rightLimelight,
      Limelight groundLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.turretLimelight = turretLimelight;
    this.leftLimelight = leftLimelight;
    this.rightLimelight = rightLimelight;
    this.groundLimelight = groundLimelight;
    this.turretBuffer = TimeInterpolatableBuffer.createDoubleBuffer(2.0);
  }

  public void addTurretObservation(double timestamp, double angle, double turretAngularVelocity) {
    this.turretBuffer.addSample(timestamp, MathHelpers.angleModulus(angle));
    this.turretLimelight.sendImuData(
        MathHelpers.angleModulus(this.robotHeading + angle),
        turretAngularVelocity + this.robotAngularVelocity,
        0.0,
        0.0,
        0.0,
        0.0);
  }

  public OptionalTagResult getLeftLimelightTagResult() {
    return leftResult;
  }

  public OptionalTagResult getRightLimelightTagResult() {
    return rightResult;
  }

  public OptionalTagResult getShooterLimelightTagResult() {
    return shooterResult;
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void hubTagsRequest() {
    if (getState() == VisionState.WAITING_FOR_HUB_TAGS || getState() == VisionState.HUB_TAGS) {
      return;
    }

    setStateFromRequest(VisionState.WAITING_FOR_HUB_TAGS);
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = MathHelpers.angleModulus(robotHeading);
    // Send IMU data to all limelights
    turretLimelight.sendImuData(this.robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    leftLimelight.sendImuData(this.robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
    rightLimelight.sendImuData(this.robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);
  }

  public void setRobotVelocity(double velocity) {
    turretLimelight.setRobotVelocity(velocity);
    leftLimelight.setRobotVelocity(velocity);
    rightLimelight.setRobotVelocity(velocity);
  }

  public void tagsRequest() {
    setStateFromRequest(VisionState.TAGS);
  }

  @Override
  public void whileInState(VisionState currentState) {
    DogLog.log("Vision/SeeingTag", seeingTag);
  }

  private Optional<Double> getAngleAtTimestamp(double timestamp) {
    return this.turretBuffer.getSample(timestamp);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        turretLimelight.setState(LimelightState.TAGS);
        leftLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);

        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case HUB_TAGS -> {
        turretLimelight.setState(LimelightState.HUB_TAGS);
        leftLimelight.setState(LimelightState.HUB_TAGS);
        rightLimelight.setState(LimelightState.HUB_TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case WAITING_FOR_HUB_TAGS -> {
        turretLimelight.setState(LimelightState.TAGS);
        leftLimelight.setState(LimelightState.TAGS);
        rightLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      default -> {}
    }
  }

  @Override
  protected void collectInputs() {
    robotAngularVelocity = imu.getRobotAngularVelocity();

    shooterResult = turretLimelight.getTagResult();
    leftResult = leftLimelight.getTagResult();
    rightResult = rightLimelight.getTagResult();

    if (shooterResult.isPresent() || leftResult.isPresent() || rightResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }

    seeingHubTags =
        seeingHubTagDebouncer.calculate(
            turretLimelight.seeingHubTag()
                || leftLimelight.seeingHubTag()
                || rightLimelight.seeingHubTag());
  }

  @Override
  protected VisionState getNextState(VisionState currentState) {
    return switch (currentState) {
      case HUB_TAGS, WAITING_FOR_HUB_TAGS -> {
        if (seeingHubTags) {
          yield VisionState.HUB_TAGS;
        }
        yield VisionState.WAITING_FOR_HUB_TAGS;
      }
      default -> currentState;
    };
  }
}
