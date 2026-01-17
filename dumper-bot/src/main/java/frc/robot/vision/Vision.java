package frc.robot.vision;

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
import frc.robot.config.DSOptions;
import frc.robot.imu.Imu;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.OptionalTagResult;
import java.util.Optional;

public class Vision extends StateMachineSubsystem<VisionState> {
  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer = new Debouncer(5.0, DebounceType.kFalling);

  private final Imu imu;
  private final Limelight mainLimelight;

  private OptionalTagResult mainResult = new OptionalTagResult();

  private double robotHeading;

  private double angularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public Vision(Imu imu, Limelight mainLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.mainLimelight = mainLimelight;
  }

  @Override
  protected void collectInputs() {
    angularVelocity = imu.getRobotAngularVelocity();

    mainResult = mainLimelight.getTagResult();

    if (mainResult.isPresent()) {
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

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }

  public OptionalTagResult getMainLimelighTagResult() {
    return mainResult;
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
    setStateFromRequest(state);
  }

  @Override
  protected void afterTransition(VisionState newState) {
    switch (newState) {
      case TAGS -> {
        mainLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    mainLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);

    DogLog.log("Vision/SeeingTag", seeingTag);
    DogLog.log("Vision/SeeingTagLast5Seconds", seenTagRecentlyForReset);
  }

  public boolean isAnyCameraOffline() {
    return mainLimelight.getCameraHealth() == CameraHealth.OFFLINE;
  }

  public boolean isAnyCameraOnlineForTags() {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return mainLimelight.isOnlineForTags();
  }
}
