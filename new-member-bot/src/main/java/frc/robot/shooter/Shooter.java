package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX bottomMotor;
  private final TalonFX topMotor;

  private double leftCurrent;
  private double rightCurrent;
  private double bottomCurrent;
  private double topCurrent;

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Current> bottomSupplyCurrentSignal;
  private final StatusSignal<Current> topSupplyCurrentSignal;

  public Shooter(TalonFX leftMotor, TalonFX rightMotor, TalonFX bottomMotor, TalonFX topMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.bottomMotor = bottomMotor;
    this.topMotor = topMotor;

    leftMotor.getConfigurator().apply(ShooterConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(ShooterConfig.RIGHT_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(ShooterConfig.BOTTOM_MOTOR_CONFIG);
    topMotor.getConfigurator().apply(ShooterConfig.TOP_MOTOR_CONFIG);

    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent();
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent();
    bottomSupplyCurrentSignal = bottomMotor.getSupplyCurrent();
    topSupplyCurrentSignal = topMotor.getSupplyCurrent();

    TunablePid.register("Shooter/Left", leftMotor, ShooterConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Right", rightMotor, ShooterConfig.RIGHT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Bottom", bottomMotor, ShooterConfig.BOTTOM_MOTOR_CONFIG);
    TunablePid.register("Shooter/Top", topMotor, ShooterConfig.TOP_MOTOR_CONFIG);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    topMotor
        .getConfigurator()
        .apply(
            ShooterConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
