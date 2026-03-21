package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("581CANivore");
  private final CANBus rio = new CANBus();

  public final TalonFX deployMotor = new TalonFX(15, canivore);

  public final TalonFX intakeLeftMotor = new TalonFX(16, rio);
  public final TalonFX intakeRightMotor = new TalonFX(17, rio);

  public final TalonFX conveyorLeftMotor = new TalonFX(18, rio);
  public final TalonFX conveyorRightMotor = new TalonFX(19, rio);

  public final TalonFX extender = new TalonFX(20);

  public final TalonFX leftFeederMotor = new TalonFX(21, rio);
  public final TalonFX rightFeederMotor = new TalonFX(22, rio);

  public final TalonFX shooterHoodMotor = new TalonFX(23, rio);

  public final TalonFX shooterLeftMotor = new TalonFX(24, rio);
  public final TalonFX shooterRightMotor = new TalonFX(25, rio);
  public final TalonFX shooterMiddleMotor = new TalonFX(26, rio);

  public final CANrange hopperCANRange = new CANrange(28, canivore);
  // TODO: get channel based on placement
  public final DigitalInput towerSensor = new DigitalInput(8);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          CompTunerConstants.DrivetrainConstants,
          CompTunerConstants.FrontLeft,
          CompTunerConstants.FrontRight,
          CompTunerConstants.BackLeft,
          CompTunerConstants.BackRight);
}
