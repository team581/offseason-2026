package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.Base581Robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.BuildConstants;

public class Robot extends Base581Robot {
  private XboxController xbox = new XboxController(0);
  private TalonFX intakeMotor = new TalonFX(16);

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
  public void robotPeriodic() {
    super.robotPeriodic();
    if (xbox.getXButton()) {
      intakeMotor.setVoltage(3.0);
    } else if (xbox.getYButton()) {
      intakeMotor.setVoltage(-2.0);
    } else if (xbox.getAButton()) {
      intakeMotor.setVoltage(0.0);
    }
  }

  @Override
  protected void configureBindings() {}
}
