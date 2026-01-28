package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  private final CANBus canivore = new CANBus("581CANivore");
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  public final TalonFX intakeMotor = new TalonFX(15, canivore);
    public final TalonFX hopperMotor = new TalonFX(16, canivore);
  public final TalonFX leftShooterMotor = new TalonFX(17, canivore);
  public final TalonFX rightShooterMotor = new TalonFX(18, canivore);
  public final TalonFX kickerShooterMotor = new TalonFX(19, canivore);
  public final TalonFX feederMotor = new TalonFX(20, canivore);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);
}
