package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.BuildConstants;
import frc.robot.hopper.Hopper;
import frc.robot.intake.Intake;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();
  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));
  private final Swerve swerve = new Swerve(hardware.drivetrain, trailblazer);
  private final Intake intake = new Intake(hardware.intakeMotor);
  private final Hopper hopper = new Hopper(hardware.hopperMotor);
  private final Shooter shooter =
      new Shooter(hardware.leftShooterMotor, hardware.rightShooterMotor);
  private final RobotManager robotManager = new RobotManager(intake, shooter);

  @SuppressWarnings("unused") // Registers itself as a
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
    DogLog.log("Robot/LeftTriggerAxis", hardware.driverController.getLeftTriggerAxis());
    var leftX = hardware.driverController.getLeftX();
    var leftY = -hardware.driverController.getLeftY();
    var rightX = hardware.driverController.getRightX();

    var translationMagnitude = ControllerHelpers.getJoystickMagnitude(leftX, leftY, 2);
    var rotationMagnitude =
        Math.copySign(ControllerHelpers.getJoystickMagnitude(rightX, 0, 5), rightX);
    swerve.setTeleopInputs(
        translationMagnitude, MathHelpers.rotation2d(leftX, leftY), rotationMagnitude);

    if (hardware.driverController.getLeftTriggerAxis() > 0.5) {
      robotManager.intakeRequest();
    } else {
      robotManager.cancelIntakeRequest();
    }

    if (hardware.driverController.getRightTriggerAxis() > 0.5) {
      robotManager.confirmShotRequest();
    } else {
      robotManager.idleRequest();
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
      robotManager.idleRequest();
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
