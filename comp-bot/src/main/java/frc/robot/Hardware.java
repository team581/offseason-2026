package frc.robot;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  public final TalonFX shooterHoodMotor = new TalonFX(0);
  public final TalonFX shooterLeftMotor = new TalonFX(0);
  public final TalonFX shooterRightMotor = new TalonFX(0);
  public final TalonFX intakeMotor = new TalonFX(0);
  public final TalonFX deployMotor = new TalonFX(0);
  public final TalonFX rotorMotor = new TalonFX(0);
  public final TalonFX horizontalMotor = new TalonFX(0);
  public final TalonFX verticalMotor = new TalonFX(0);
  public final CANdle candle = new CANdle(0);
  public final TalonFX turretMotor = new TalonFX(0);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);
}
