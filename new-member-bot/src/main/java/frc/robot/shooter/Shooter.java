package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private final TalonFX topleftMotor;
  private final TalonFX toprightMotor;
  private final TalonFX bottomleftMotor;
  private final TalonFX bottomrightMotor;
  private final Follower topleftFollower;
  private final Follower bottomleftFollower;
  private final Follower
      bottomrightFollower; // FYI:L=left and R=right and then the rest pretty self explanitory

  private double topleftCurrent;
  private double toprightCurrent;
  private double bottomleftCurrent;
  private double bottomrightCurrent;
  private double topleftVoltage;
  private double toprightVoltage;
  private double bottomleftVoltage;
  private double bottomrightVoltage;

  private double topleftMotorRpm = 0;
  private double toprightMotorRpm = 0;
  private double bottomleftMotorRpm = 0;
  private double bottomrightMotorRpm = 0;
  private double shootingRpm = 0;
  private double scoreDistance = 0;
  private double feedingRpm = 0;
  private double feedDistance = 0;
  private boolean AtGoal = false;
  // Values can be assigned and adjusted if needed later on
  private final StatusSignal<Current> topleftSupplyCurrentSignal;
  private final StatusSignal<Current> toprightSupplyCurrentSignal;
  private final StatusSignal<Current> bottomleftSupplyCurrentSignal;
  private final StatusSignal<Current> bottomrightSupplyCurrentSignal;

  private final StatusSignal<Voltage> topleftVoltageSignal;
  private final StatusSignal<Voltage> toprightVoltageSignal;
  private final StatusSignal<Voltage> bottomleftVoltageSignal;
  private final StatusSignal<Voltage> bottomrightVoltageSignal;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public Shooter(
      TalonFX topleftMotor,
      TalonFX toprightMotor,
      TalonFX bottomleftMotor,
      TalonFX bottomrightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);
    this.topleftMotor = topleftMotor;
    this.toprightMotor = toprightMotor;
    this.bottomleftMotor = bottomleftMotor;
    this.bottomrightMotor = bottomrightMotor;

    this.topleftFollower = new Follower(toprightMotor.getDeviceID(), MotorAlignmentValue.Aligned);
    this.bottomleftFollower =
        new Follower(toprightMotor.getDeviceID(), MotorAlignmentValue.Aligned);
    this.bottomrightFollower =
        new Follower(
            toprightMotor.getDeviceID(),
            MotorAlignmentValue.Aligned); // not sure what alignment we would want

    topleftMotor.setControl(topleftFollower);
    bottomleftMotor.setControl(bottomleftFollower);
    bottomrightMotor.setControl(bottomrightFollower);

    topleftMotor.getConfigurator().apply(ShooterConfig.TOP_LEFT_MOTOR_CONFIG);
    toprightMotor.getConfigurator().apply(ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    bottomleftMotor.getConfigurator().apply(ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    bottomrightMotor.getConfigurator().apply(ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);

    topleftSupplyCurrentSignal = topleftMotor.getSupplyCurrent();
    toprightSupplyCurrentSignal = toprightMotor.getSupplyCurrent();
    bottomleftSupplyCurrentSignal = bottomleftMotor.getSupplyCurrent();
    bottomrightSupplyCurrentSignal = bottomrightMotor.getSupplyCurrent();

    topleftVoltageSignal = topleftMotor.getMotorVoltage();
    toprightVoltageSignal = toprightMotor.getMotorVoltage();
    bottomrightVoltageSignal = bottomleftMotor.getMotorVoltage();
    bottomleftVoltageSignal = bottomrightMotor.getMotorVoltage();

    TunablePid.register("Shooter/TopLeft", topleftMotor, ShooterConfig.TOP_LEFT_MOTOR_CONFIG);
    TunablePid.register("Shooter/TopRight", toprightMotor, ShooterConfig.TOP_RIGHT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/BottomLeft", bottomleftMotor, ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG);
    TunablePid.register(
        "Shooter/BottomRight", bottomrightMotor, ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topleftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.TOP_LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    toprightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.TOP_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomleftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomrightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.BOTTOM_RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEEDING);
  }

  public double getFeedTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_TOF_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEED_TOF.get(distance);
  }

  public double getScoreTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_TOF_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_TOF.get(distance);
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

    DogLog.log("Shooter/TopLeft/RPM", topleftMotorRpm);
    DogLog.log("Shooter/TopRight/RPM", toprightMotorRpm);
    DogLog.log("Shooter/BottomLeft/RPM", bottomleftMotorRpm);
    DogLog.log("Shooter/BottomRight/RPM", bottomrightMotorRpm);

    switch (state) {
      case IDLE -> {
        toprightMotor.setControl(velocityRequest.withVelocity(ShooterConfig.IDLE_RPM / 60.0));
        DogLog.log("Shooter/IdleRPM", ShooterConfig.IDLE_RPM);
      }

      case HUB_SCORING -> {
        toprightMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
        DogLog.log("Shooter/ScoringRPM", shootingRpm);
      }

      case FEEDING -> {
        toprightMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
        DogLog.log("Shooter/FeedingRPM", feedingRpm);
      }
    }
  }

  private double distanceToFeedingRpm(double distance) {
    return distance * 0.0; // can be given a value instead of 0
  }

  private double distanceToScoringRpm(double distance) {
    return distance * 0.0; // can be given a value instead of 0
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToScoringRpm(scoreDistance));
    feedingRpm = Math.min(ShooterConfig.MAX_SAFE_RPM, distanceToFeedingRpm(feedDistance));

    topleftCurrent = topleftSupplyCurrentSignal.getValueAsDouble();
    toprightCurrent = toprightSupplyCurrentSignal.getValueAsDouble();
    bottomleftCurrent = bottomleftSupplyCurrentSignal.getValueAsDouble();
    bottomrightCurrent = bottomrightSupplyCurrentSignal.getValueAsDouble();

    topleftMotorRpm = topleftMotor.getVelocity().getValueAsDouble() * 60.0;
    toprightMotorRpm = toprightMotor.getVelocity().getValueAsDouble() * 60.0;
    bottomleftMotorRpm = bottomleftMotor.getVelocity().getValueAsDouble() * 60.0;
    bottomrightMotorRpm = bottomrightMotor.getVelocity().getValueAsDouble() * 60.0;

    AtGoal = MathUtil.isNear(toprightMotorRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE);
  }
}
