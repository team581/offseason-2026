package frc.robot.power_manager;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.GenericDeploy;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PowerManager extends StateMachineSubsystem<PowerManagerState> {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  private final PowerManaged shooter;
  private final PowerManaged intake;
  private final PowerManaged deploy;
  private final PowerManaged shooterHood;
  private final PowerManaged feeder;
  private final PowerManaged conveyor;
  private final PowerManaged swerve;

  public PowerManager(
      Shooter shooter,
      Intake intake,
      GenericDeploy deploy,
      ShooterHood shooterHood,
      Feeder feeder,
      Conveyor conveyor,
      Swerve swerve) {
    super(SubsystemPriority.POWER_MANAGER, PowerManagerState.IDLE);
    this.shooter = shooter;
    this.intake = intake;
    this.deploy = deploy;
    this.shooterHood = shooterHood;
    this.feeder = feeder;
    this.conveyor = conveyor;
    this.swerve = swerve;
  }

  public void firstAutoSegmentRequest() {
    setStateFromRequest(PowerManagerState.AUTO_FIRST_SEGMENT);
  }

  public void idleRequest() {
    setStateFromRequest(PowerManagerState.IDLE);
  }

  public void shootingRequest() {
    setStateFromRequest(PowerManagerState.SHOOTING);
  }

  public void turboRequest() {
    setStateFromRequest(PowerManagerState.TURBO_MODE);
  }

  @Override
  protected void afterTransition(PowerManagerState newState) {
    executor.execute(
        () -> {
          DogLog.timestamp("PowerManager/UpdatedCurrentsAt");
          shooter.applyCurrentLimits(newState.shooterSupplyCurrent);
          intake.applyCurrentLimits(newState.intakeSupplyCurrent);
          deploy.applyCurrentLimits(newState.deploySupplyCurrent);
          shooterHood.applyCurrentLimits(newState.shooterHoodSupplyCurrent);
          feeder.applyCurrentLimits(newState.feederSupplyCurrent);
          conveyor.applyCurrentLimits(newState.conveyorSupplyCurrent);
          swerve.applyCurrentLimits(newState.swerveSupplyCurrent);
        });
  }
}
