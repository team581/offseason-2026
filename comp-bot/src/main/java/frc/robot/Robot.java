package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.autos.Autos;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.Imu;
import frc.robot.localization.Localization;
import frc.robot.robot_manager.RobotManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));

  private final Swerve swerve = new Swerve(hardware.drivetrain, trailblazer);
  private final Imu imu = new Imu(swerve.drivetrain);

  private final Localization localization = new Localization(swerve, hardware.drivetrain, imu);

  private final ShooterHood shooterHood = new ShooterHood(hardware.shooterHoodMotor);

  private final Shooter shooter =
      new Shooter(hardware.shooterrightMotor, hardware.shooterleftMotor);

  private final RobotManager robotManager = new RobotManager(shooterHood, localization, swerve);

  @SuppressWarnings("unused") // Registers itself as a subsystem
  private final Autos autos = new Autos(robotManager, trailblazer);

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
  public void teleopPeriodic() {
    var leftX = hardware.driverController.getLeftX();
    var leftY = -hardware.driverController.getLeftY();
    var rightX = hardware.driverController.getRightX();

    var translationMagnitude = ControllerHelpers.getJoystickMagnitude(leftX, leftY, 2);
    var rotationMagnitude =
        Math.copySign(ControllerHelpers.getJoystickMagnitude(rightX, 0, 5), rightX);

    swerve.setTeleopInputs(
        translationMagnitude, MathHelpers.rotation2d(leftX, leftY), rotationMagnitude);
  }

  @Override
  protected void configureBindings() {
    var driverBack = enabledEvent.and(hardware.driverController.back(buttonBindingsLoop));
    driverBack.rising().ifHigh(localization::zeroGyro);

    var driverY = enabledEvent.and(hardware.driverController.y(buttonBindingsLoop));
    driverY.rising().ifHigh(shooter::scoreRequest);
  }
}
