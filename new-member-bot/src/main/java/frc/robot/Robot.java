package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.Base581Robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.BuildConstants;

public class Robot extends Base581Robot {
  private XboxController xbox = new XboxController(0);
  private TalonFX intakeMotorR = new TalonFX(16);
  private TalonFX intakeMotorL = new TalonFX(17);

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
      intakeMotorR.setVoltage(3.0);
      intakeMotorL.setVoltage(3.0);
    } else if (xbox.getYButton()) {
      intakeMotorR.setVoltage(-2.0);
      intakeMotorL.setVoltage(-2.0);

    } else if (xbox.getAButton()) {
      intakeMotorR.setVoltage(0.0);
      intakeMotorL.setVoltage(0.0);
    }

    double axis = xbox.getRawAxis(0);
    if (xbox.getRawAxis(0) > 0 || xbox.getRawAxis(0) < 0) {
      intakeMotorR.setVoltage(12.0 * axis);
      intakeMotorL.setVoltage(12.0 * axis);
    } else {
      intakeMotorR.setVoltage(0.0);
      intakeMotorL.setVoltage(0.0);
    }
  }

  @Override
  protected void configureBindings() {}
}
