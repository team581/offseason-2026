package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.BuildConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.robot_manager.RobotManager;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final IntakeSubsystem intake = new IntakeSubsystem(hardware.intakeMotor);
  private final RobotManager robotManager = new RobotManager(intake);
  

  @SuppressWarnings("unused") // Registers itself as a subsystem
  // private final Autos autos = new Autos(robotManager, trailblazer);

  public Robot() {
    logMetadata(
        BuildConstants.MAVEN_NAME,
        BuildConstants.BUILD_DATE,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_DATE,
        BuildConstants.GIT_BRANCH,
        BuildConstants.DIRTY);

    finalizeInit();
  }

  @Override
  protected void configureBindings() {}

  @Override
  public void teleopPeriodic() {
    
    
    

    
    

    // swerve.setTeleopInputs(
    //     translationMagnitude, MathHelpers.rotation2d(leftX, leftY), rotationMagnitude);

    if (hardware.driverController.getLeftTriggerAxis() > 0.5) {
      robotManager.intakeRequest();
    } else {
      robotManager.cancelIntakeRequest();
    }

    if (hardware.driverController.getRightTriggerAxis() > 0.5) {
      robotManager.confirmShotRequest();
    } else {
      robotManager.cancelShotRequest();
    }

    if (hardware.driverController.getXButtonPressed()) {
      robotManager.shootHubWaitRequest();
    }

    if (hardware.driverController.getAButtonPressed()) {
      robotManager.feed1WaitRequest();
    }

    if (hardware.driverController.getBButtonPressed()) {
      robotManager.feed1WaitRequest();
    }

    if (hardware.driverController.getRightBumperButtonPressed()) {
      robotManager.cancelShotRequest();
    }

    if (hardware.driverController.getYButtonPressed()) {
      robotManager.idleRequest();
    }

    if (hardware.driverController.getPOV() == 0) {
      robotManager.climbSequenceForward();
    }

    if (hardware.driverController.getPOV() == 180) {
      robotManager.climbSequenceBackward();
    }

    if (hardware.driverController.getBackButtonPressed()) {
    //   localization.zeroGyro();
    }
  }
}
