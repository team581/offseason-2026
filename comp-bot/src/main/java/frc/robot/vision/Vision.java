package frc.robot.vision;

import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.results.OptionalTagResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingHubTagDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Imu imu;
  private final Limelight backLimelight;
  private final Limelight groundLimelight;

  private OptionalTagResult backResult = new OptionalTagResult();

  private double robotHeading;

  private double robotAngularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;

  private boolean seeingHubTags = false;

  public Vision(
      Imu imu, Limelight backLimelight, Limelight groundLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.backLimelight = backLimelight;
    this.groundLimelight = groundLimelight;
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

  @Override
  protected void collectInputs() {
    robotAngularVelocity = imu.getRobotAngularVelocity();

    backResult = backLimelight.getTagResult();

    if (backResult.isPresent()) {
      hasSeenTag = true;
      seeingTag = true;
    } else {
      seeingTag = false;
    }

    seeingHubTags =
        seeingHubTagDebouncer.calculate(backLimelight.seeingHubTag());
  }

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getBackLimelightTagResult() {
    return backResult;
  }

  public boolean seeingTag() {
    return seeingTag || RobotBase.isSimulation();
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public void setState(VisionState state) {
    if (state == VisionState.HUB_TAGS && getState() == VisionState.WAITING_FOR_HUB_TAGS) {
      return;
    }
    if (state == VisionState.HUB_TAGS) {
      state = VisionState.WAITING_FOR_HUB_TAGS;
    }

    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        backLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case HUB_TAGS -> {
        backLimelight.setState(LimelightState.HUB_TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      case WAITING_FOR_HUB_TAGS -> {
        backLimelight.setState(LimelightState.TAGS);
        groundLimelight.setState(LimelightState.CLUSTER_MAP);
      }
      default -> {}
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    // Send IMU data to all limelights
    backLimelight.sendImuData(robotHeading, robotAngularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
  }
}
