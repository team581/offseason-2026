package frc.robot.vision.limelight;

import com.team581.config.CameraConfig;
import com.team581.mechanisms.vision.CameraHealth;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.limelight.LimelightHelpers;
import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import com.team581.vision.limelight.PoseEstimateValidator;
import com.team581.vision.results.OptionalTagResult;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.scheduling.SubsystemPriority;

public class Limelight extends StateMachineSubsystem<LimelightState> {
  public final String limelightTableName;
  private final String mtTimestampKey;
  public final CameraConfig config;
  private final String rawPoseKey;
  private double angularVelocity = 0.0; // As robot moves
  private double linearVelocity = 0.0; // As robot moves
  private OptionalTagResult tagResult = new OptionalTagResult();
  private final String name;
  private final PoseEstimateValidator poseEstimateValidator;
  private boolean updatedLimelightPos = false; // false or true dependant;

  private boolean latestEstimateTrusted = false; // false or true dependent;
  private PoseEstimate latestEstimate = new PoseEstimate();

  PoseEstimate mT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);

  public Limelight(String name, LimelightState initialState, CameraConfig config) {
    super(SubsystemPriority.VISION, initialState);
    limelightTimer.start();
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

  // add autonomuos too early but something to keep in mind!

  @Override
  public void robotPeriodic() {
    latestEstimate = mT1Estimate;
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
    }
  }
  // planning to add fiducial aka apriltags data according to Jonah! * REMEMBER THIS!!
}
