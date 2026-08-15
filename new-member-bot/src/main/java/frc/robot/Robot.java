package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.Base581Robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.BuildConstants;
import frc.robot.intake.Intake;

public class Robot extends Base581Robot {
  private XboxController xbox = new XboxController(0);
  private TalonFX intakeMotorR = new TalonFX(16);
  private TalonFX intakeMotorL = new TalonFX(17);
  private final Intake intake = new Intake(intakeMotorL, intakeMotorR);

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
    
    if (xbox.getLeftTriggerAxis() > 0.5) {
      intake.collectRequest();
    }else if(xbox.getRightTriggerAxis() > 0.5){
      intake.ejectRequest();
    }else{
      intake.idleRequest();
    }
  }

  @Override
  protected void configureBindings() {}
}
