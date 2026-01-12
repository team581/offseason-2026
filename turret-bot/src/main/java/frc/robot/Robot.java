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
import frc.robot.imu.ImuSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.turret.TurretSubsystem;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;

public class Robot extends Base581Robot {
  private static final double TURRET_CAMERA_HEIGHT = Units.inchesToMeters(0.0);
  private static final double TURRET_CAMERA_PITCH = 0.0;

  private final Hardware hardware = new Hardware();

  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(4.0, 0, 0)));

  private final SwerveSubsystem swerve = new SwerveSubsystem(hardware.drivetrain, trailblazer);
  private final ImuSubsystem imu = new ImuSubsystem(swerve.drivetrain);
  private final Limelight turretLimelight =
      new Limelight(
          "turret",
          LimelightState.TAGS,
          new CameraConfig(
              LimelightModel.FOUR,
              true,
              0.0,
              0.0,
              TURRET_CAMERA_HEIGHT,
              TURRET_CAMERA_PITCH,
              0.0,
              0.0));
  private final VisionSubsystem vision = new VisionSubsystem(imu, turretLimelight);
  private final LocalizationSubsystem localization =
      new LocalizationSubsystem(swerve, hardware.drivetrain, vision);
  private final TurretSubsystem turret = new TurretSubsystem(hardware.turretMotor, localization);

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
  protected void configureBindings() {}

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

    if (hardware.driverController.getBackButtonPressed()) {
      localization.zeroGyro();
    }
  }
}
