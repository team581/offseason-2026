package frc.robot.vision.limelight;

import com.team581.config.CameraConfig;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.limelight.LimelightHelpers;
import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import com.team581.vision.limelight.PoseEstimateValidator;
import com.team581.mechanisms.vision.CameraHealth;

import edu.wpi.first.wpilibj.DriverStation;

public class Limelight extends StateMachineSubsystem<LimelightState> {
  protected Limelight(SubsystemPriorityBase priority, frc.robot.vision.limelight.LimelightState initialState) {
    super(priority, initialState);
    //TODO Auto-generated constructor stub
  }

  public final String limelightTableName;
  public final CameraConfig config;
  private final String name;
  private final PoseEstimateValidator poseEstimateValidator;

  private boolean updatedLimelightPos = false;
  private boolean latestEstimateTrusted = false;


  private PoseEstimate latestEstimate = new PoseEstimate();


  PoseEstimate mT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightTableName);

  latestEstimate = mT1Estimate;

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
      }}}

   private CameraHealth getCameraHealth() {
   }}
