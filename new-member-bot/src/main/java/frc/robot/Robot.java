package frc.robot;

import com.team581.Base581Robot;
import com.team581.controller.ControllerBindings;
import frc.robot.generated.BuildConstants;
import frc.robot.intake.Intake;

public class Robot extends Base581Robot {
  private final Hardware hardware = new Hardware();
  private final Intake intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);

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

    // X = Halt Intake
    driver.x().onPress(intake::haltIntakeRequest);
  }
}
