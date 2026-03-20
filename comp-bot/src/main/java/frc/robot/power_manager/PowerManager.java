package frc.robot.power_manager;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.intake.Intake;
import frc.robot.kicker.Kicker;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PowerManager extends StateMachineSubsystem<PowerManagerState> {
  private final ExecutorService executor = Executors.newSingleThreadExecutor();
  private final PowerManaged shooter;
  private final PowerManaged intake;
  private final PowerManaged deploy;
  private final PowerManaged shooterHood;
  private final PowerManaged kicker;
  private final PowerManaged feeder;
  private final PowerManaged conveyor;

  public PowerManager(
      Shooter shooter,
      Intake intake,
      Deploy deploy,
      ShooterHood shooterHood,
      Kicker kicker,
      Feeder feeder,
      Conveyor conveyor) {
    super(SubsystemPriority.POWER_MANAGER, PowerManagerState.IDLE);
    this.shooter = shooter;
    this.intake = intake;
    this.deploy = deploy;
    this.shooterHood = shooterHood;
    this.kicker = kicker;
    this.feeder = feeder;
    this.conveyor = conveyor;
  }

  public void idleRequest() {
    setStateFromRequest(PowerManagerState.IDLE);
  }

  public void shootingRequest() {
    setStateFromRequest(PowerManagerState.SHOOTING);
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
          kicker.applyCurrentLimits(newState.kickerSupplyCurrent);
          feeder.applyCurrentLimits(newState.feederSupplyCurrent);
          conveyor.applyCurrentLimits(newState.conveyorSupplyCurrent);
        });
  }
}
