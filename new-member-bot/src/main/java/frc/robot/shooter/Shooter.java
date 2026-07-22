package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private final TalonFX LtopMotor;
  private final TalonFX RtopMotor;
  private final TalonFX LbottomMotor;
  private final TalonFX RbottomMotor;

  private final Follower LtopFollower;
  private final Follower LbottomFollower;
  private final Follower RbottomFollower;

  // FYI:L=left and R=right and then the rest pretty self explanitory
  private double LtopCurrent;
  private double RtopCurrent;
  private double LbottomCurrent;
  private double RbottomCurrent;

  private double LtopVoltage;
  private double RtopVoltage;
  private double LbottomVoltage;
  private double RbottomVoltage;

  // Values can be assigned and adjusted if needed later on
  private double LtopMotorRpm = 0;
  private double RtopMotorRpm = 0;
  private double LbottomMotorRpm = 0;
  private double RbottomMotorRpm = 0;
  private double shootingRpm = 0;
  private double scoreDistance = 0;
  private double feedingRpm = 0;
  private double feedDistance = 0;
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

    this.LtopFollower =
        new Follower(RtopMotor.getDeviceID(), null); // not sure what alignment we would want
    this.LbottomFollower =
        new Follower(RtopMotor.getDeviceID(), null); // not sure what alignment we would want
    this.RbottomFollower =
        new Follower(RtopMotor.getDeviceID(), null); // not sure what alignment we would want

    LtopMotor.setControl(LtopFollower);
    LbottomMotor.setControl(LbottomFollower);
    RbottomMotor.setControl(RbottomFollower);

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

  @Override
  public void whileInState(ShooterState state) {
    DogLog.log("Shooter/TopLeft/RPM", LtopMotorRpm);
    DogLog.log("Shooter/BottomLeft/RPM", LbottomMotorRpm);
    DogLog.log("Shooter/BottomRight/RPM", RbottomMotorRpm);
    DogLog.log("Shooter/TopRight/RPM", RtopMotorRpm);
  }

  @Override
  protected void collectInputs() {
    // shootingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    // feedingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));
    // fix later
  }
}
