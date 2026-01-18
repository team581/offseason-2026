package frc.robot;

import com.team581.Base581Robot;
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import com.team581.controller.ControllerHelpers;
import com.team581.math.MathHelpers;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.Autos;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.Imu;
import frc.robot.localization.Localization;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;

public class Robot extends Base581Robot {
  private static final double TURRET_CAMERA_HEIGHT = Units.inchesToMeters(30.75);
  private static final double TURRET_CAMERA_PITCH = 0.0;

  private final Hardware hardware = new Hardware();

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));

  private final Swerve swerve = new Swerve(hardware.drivetrain, trailblazer);
  private final Imu imu = new Imu(swerve.drivetrain);
  private final Limelight turretLimelight =
      new Limelight(
          "turret",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.FOUR,
              false,
              0.0,
              0.0,
              TURRET_CAMERA_HEIGHT,
              TURRET_CAMERA_PITCH,
              0.0,
              0.0));
  private final Vision vision = new Vision(imu, turretLimelight);
  private final Localization localization = new Localization(swerve, hardware.drivetrain, vision);
  private final Turret turret = new Turret(hardware.turretMotor);

  private final RobotManager robotManager = new RobotManager(localization, swerve, turret, vision);

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
  protected void configureBindings() {
    var driverBack = hardware.driverController.back(buttonBindingsLoop);
    driverBack.rising().ifHigh(localization::zeroGyro);

    var driverY = hardware.driverController.y(buttonBindingsLoop);
    driverY.rising().ifHigh(robotManager::hubAimRequest);

    var driverA = hardware.driverController.a(buttonBindingsLoop);
    driverA.rising().ifHigh(robotManager::lockForwardRequest);

    var driverX = hardware.driverController.x(buttonBindingsLoop);
    driverX.rising().ifHigh(turret::homeRequest);

    var driverB = hardware.driverController.b(buttonBindingsLoop);
    driverB.rising().ifHigh(robotManager::tagAimRequest);
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
}
