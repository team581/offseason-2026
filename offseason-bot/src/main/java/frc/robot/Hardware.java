package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;

public class Hardware {
  public final XboxController driverController = new XboxController(0);
  public final XboxController operatorController = new XboxController(1);

  private final CANBus canivore = new CANBus("581CANivore");
  private final CANBus rio = CANBus.roboRIO();

  public final TalonFX deployLeftMotor = new TalonFX(15, canivore);
  public final TalonFX deployRightMotor = new TalonFX(30, canivore);

  public final TalonFX intakeLeftMotor = new TalonFX(16, rio);
  public final TalonFX intakeRightMotor = new TalonFX(17, rio);

  public final TalonFX conveyorMotor = new TalonFX(18, canivore);
  public final TalonFX feederMotor = new TalonFX(19, canivore);

  public final TalonFX tunnelTopMotor = new TalonFX(20, canivore);
  public final TalonFX tunnelBottomMotor = new TalonFX(21, canivore);

  public final TalonFX shooterHoodMotor = new TalonFX(22, canivore);

  public final TalonFX shooterLeftMotor = new TalonFX(23, canivore);
  public final TalonFX shooterRightMotor = new TalonFX(24, canivore);

  public final TalonFX funnelerMotor = new TalonFX(27, canivore);

  public final TalonFX turretMotor = new TalonFX(28, canivore);

  public final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);
}
