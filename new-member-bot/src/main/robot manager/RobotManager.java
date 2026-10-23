package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.health.HealthManager;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Shooter shooter;
  // private final ShooterHood shooterHood;//add when time comes
  public final Swerve swerve;
  // public final HopperManager hopperMananger;//add when time comes
  public final HealthManager healthManager;
  public final Localization localization;
  // public final PowerManager powerManager;// add when time comes
  public final XboxController driverController;
  private AimingParameters scoringParameters = new AimingParameters(0, 0, 0, 0);
  private AimingParameters feedingParameters = new AimingParameters(0, 0, 0, 0);

  public RobotManager(
      HopperManager hopperManager,
      Localization localization,
      Swerve swerve,
      Shooter shooter,
      XboxController driverController,
      HealthManager health,
      Hardware hardware) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.hopperManager = hopperManager;
    this.localization = localization;
    this.swerve = swerve;
    this.shooter = shooter;
    this.driverController = driverController;
    this.health = health;
    this.hardware = hardware;
    this.powerManager = powerManager;
  }
}
