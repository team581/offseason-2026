package frc.robot.vision;

import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import frc.robot.vision.results.OptionalTagResult;

public class VisionSubsystem extends StateMachineSubsystem<VisionState> {
private static final Transform2d TURRET_TO_CAMERA = new Transform2d(0.0,0.0,Rotation2d.kZero);


  private final Debouncer seeingTagDebouncer = new Debouncer(1.0, DebounceType.kFalling);
  private final Debouncer seeingTagForPoseResetDebouncer =
      new Debouncer(5.0, DebounceType.kFalling);

  private final ImuSubsystem imu;
  private final Limelight turretLimelight;

  private OptionalTagResult turretResult = new OptionalTagResult();
  private OptionalTagResult adjustedTurretResult = new OptionalTagResult();


  private double robotHeading;

  private double angularVelocity;

  private boolean hasSeenTag = false;
  private boolean seeingTag = false;
  private boolean seeingTagDebounced = false;
  private boolean seenTagRecentlyForReset = true;

  public VisionSubsystem(ImuSubsystem imu, Limelight turretLimelight) {
    super(SubsystemPriority.VISION, VisionState.TAGS);
    this.imu = imu;
    this.turretLimelight = turretLimelight;
  }

  @Override
  protected void collectInputs() {
    angularVelocity = imu.getRobotAngularVelocity();

    turretResult = turretLimelight.getTagResult();

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

  public void setEstimatedPoseAngle(double robotHeading) {
    this.robotHeading = robotHeading;
  }


  public OptionalTagResult getTurretLimelighTagResult() {
    return turretResult;
  }

  public OptionalTagResult getAdjustedTurretLimelightTagResult() {
    if (turretResult.isEmpty()) {
      return adjustedTurretResult.empty();
    }

    return adjustedTurretResult.empty();
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
        turretLimelight.setState(LimelightState.TAGS);
      }
    }
  }

  @Override
  public void whileInState(VisionState currentState) {
    turretLimelight.sendImuData(robotHeading, angularVelocity, 0.0, 0.0, 0.0, 0.0);

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
    return turretLimelight.isOnlineForTags();
  }
}
