package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerBindings;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.BuildConstants;
import frc.robot.health.HealthManager;
import frc.robot.intake.Conveyor.Conveyor;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.swerve.Swerve;

public class Robot extends Base581Robot {
  private final HealthManager healthManager = new HealthManager();

  private final Hardware hardware = new Hardware();
  private final Intake intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
  private final Deploy deploy = new Deploy(hardware.deployDifferentialMechanism);
  private final Shooter shooter =
      new Shooter(
          hardware.shooterTopLeftMotor,
          hardware.shooterTopRightMotor,
          hardware.shooterBottomLeftMotor,
          hardware.shooterBottomRightMotor);
  private final Conveyor conveyor =
      new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);
  private final Swerve swerve =
      new Swerve(
          hardware.drivetrain,
          healthManager,
          hardware.driverController,
          new Trailblazer(
              new HeuristicPathTracker(new PoseErrorTolerance(kDefaultPeriod)),
              new PidPathFollower(
                  new PIDController(kDefaultPeriod, kDefaultPeriod, kDefaultPeriod),
                  new PIDController(kDefaultPeriod, kDefaultPeriod, kDefaultPeriod))));

  // 1. Create an Intake instance, we wrote the class but need to actually use it + do something
  // with it
  // 2. Create an XboxController instance, that lets us run actions when buttons are clicked
  // 3. In the configureBindings() function, add in a binding for each intake state to a button (ex.
  // ABXY)
  // 4. Make sure it compiles, deploy it, then test it
  // 5. profit

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
    var driver =
        new ControllerBindings(buttonBindingsLoop, enabledEvent, hardware.driverController);
    // A = Intake
    driver.a().onPress(intake::intakeRequest);

    // B = Eject
    driver.b().onPress(intake::ejectRequest);

    operator
        .leftTrigger()
        .onPress(robotManager::stowDeployRequest)
        .onRelease(robotManager::cancelStowDeployRequest);

    operator.b().onPress(deploy::fixAlignmentRequest).onRelease(deploy::intakeRequest);

    // X = Halt Intake
    driver.x().onPress(intake::haltIntakeRequest);

    // Y = Spin to Score
    driver.y().onPress(() -> shooter.hubscoreRequest(14.0));

    // Left Bumper = Spin to Feed
    driver.leftBumper().onPress(() -> shooter.feedRequest(14.0));

    // Right Bumper = Spin to Idle
    driver.rightBumper().onPress(shooter::idleRequest);
    driver
        .start()
        .onPress(
            () -> hardware.drivetrain.seedFieldCentric()); // Make Button Binding to home the Gyro
  }
}
