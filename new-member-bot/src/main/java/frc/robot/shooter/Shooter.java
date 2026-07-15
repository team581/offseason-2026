package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private final TalonFX LtopMotor;
  private final TalonFX RtopMotor;
  private final TalonFX LbottomMotor;
  private final TalonFX RbottomMotor;

  // private final Follower leftFollower; not sure
  // private final Follower rightFollower; not sure

  // FYI:L=left and R=right and then the rest pretty self explanitory
  private double LtopCurrent;
  private double RtopCurrent;
  private double LbottomCurrent;
  private double RbottomCurrent;

  private double LtopVoltage;
  private double RtopVoltage;
  private double LbottomVoltage;
  private double RbottomVoltage;

  private double scoreDistance = 0; // TBD
  private double feedDistance = 0; // TBD
  private boolean AtGoal;

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;
  private final StatusSignal<Current> bottomSupplyCurrentSignal;
  private final StatusSignal<Current> topSupplyCurrentSignal;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public Shooter(
      TalonFX topleftMotor,
      TalonFX toprightMotor,
      TalonFX bottomleftMotor,
      TalonFX bottomrightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);
    this.LtopMotor = topleftMotor;
    this.RtopMotor = toprightMotor;
    this.LbottomMotor = bottomleftMotor;
    this.RbottomMotor = bottomrightMotor;

    LtopMotor.getConfigurator().apply(ShooterConfig.TOP_LEFT_MOTOR_CONFIG);
    RtopMotor.getConfigurator().apply(ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    LbottomMotor.getConfigurator().apply(ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    RbottomMotor.getConfigurator().apply(ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);

    leftSupplyCurrentSignal = LtopMotor.getSupplyCurrent();
    rightSupplyCurrentSignal = RtopMotor.getSupplyCurrent();
    bottomSupplyCurrentSignal = LbottomMotor.getSupplyCurrent();
    topSupplyCurrentSignal = RbottomMotor.getSupplyCurrent();

    TunablePid.register("Shooter/Left", LtopMotor, ShooterConfig.TOP_LEFT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Right", RtopMotor, ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Bottom", LbottomMotor, ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Top", RbottomMotor, ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    LtopMotor.getConfigurator()
        .apply(
            ShooterConfig.TOP_LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    RtopMotor.getConfigurator()
        .apply(
            ShooterConfig.TOP_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    LbottomMotor.getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    RbottomMotor.getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEEDING);
  }

  public void hubscoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.HUB_SCORING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }
}
// @Override
// protected void collectInputs() {

// } //TODO
