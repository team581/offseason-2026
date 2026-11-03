package frc.robot.power_manager;

import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.feeder.Feeder;
import frc.robot.funneler.Funneler;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
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
  private final PowerManaged funneler;
  private final PowerManaged swerve;
  private final PowerManaged turret;

  public PowerManager(
      Shooter shooter,
      Intake intake,
      Deploy deploy,
      ShooterHood shooterHood,
      Feeder feeder,
      Conveyor conveyor,
      Funneler funneler,
      Swerve swerve,
      Turret turret) {
    super(SubsystemPriority.POWER_MANAGER, PowerManagerState.IDLE);
    this.shooter = shooter;
    this.intake = intake;
    this.deploy = deploy;
    this.shooterHood = shooterHood;
    this.feeder = feeder;
    this.conveyor = conveyor;
    this.funneler = funneler;
    this.swerve = swerve;
    this.turret = turret;
  }

  public void beastModeRequest() {
    setStateFromRequest(PowerManagerState.BEAST_MODE);
  }

  public void feedingFarRequest() {
    setStateFromRequest(PowerManagerState.FEEDING_FAR);
  }

  public void feedingRequest() {
    setStateFromRequest(PowerManagerState.FEEDING);
  }

  public void firstAutoSegmentRequest() {
    setStateFromRequest(PowerManagerState.AUTO_FIRST_SEGMENT);
  }

  public void idleRequest() {
    setStateFromRequest(PowerManagerState.IDLE);
  }

  public void prioritizeIntakeRequest() {
    setStateFromRequest(PowerManagerState.PRIORITIZE_INTAKE);
  }

  public void scoringFarRequest() {
    setStateFromRequest(PowerManagerState.SCORING_FAR);
  }

  public void scoringRequest() {
    setStateFromRequest(PowerManagerState.SCORING);
  }

  public void turboRequest() {
    setStateFromRequest(PowerManagerState.TURBO_MODE);
  }

  @Override
  protected void afterTransition(PowerManagerState newState) {
    DogLog.timestamp("PowerManager/UpdatedCurrentsAt");
    executor.execute(() -> {
      shooter.applyCurrentLimits(newState.shooterSupplyCurrent.get());
      intake.applyCurrentLimits(newState.intakeSupplyCurrent.get());
      deploy.applyCurrentLimits(newState.deploySupplyCurrent.get());
      shooterHood.applyCurrentLimits(newState.shooterHoodSupplyCurrent.get());
      feeder.applyCurrentLimits(newState.feederSupplyCurrent.get());
      conveyor.applyCurrentLimits(newState.conveyorSupplyCurrent.get());
      funneler.applyCurrentLimits(newState.funnelerSupplyCurrent.get());
      swerve.applyCurrentLimits(newState.swerveSupplyCurrent.get());
      turret.applyCurrentLimits(newState.turretSupplyCurrent.get());
    });
  }
}
