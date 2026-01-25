package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  public final TalonFX shooterHoodMotor = new TalonFX(0);
  public final TalonFX shooterleftMotor = new TalonFX(0);
  public final TalonFX shooterrightMotor = new TalonFX(0);
  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);
}
