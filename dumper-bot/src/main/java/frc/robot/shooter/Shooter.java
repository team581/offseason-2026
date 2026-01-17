package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private static final double MAX_SAFE_RPM = 4000;

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 1000.0), Map.entry(2.0, 2500.0), Map.entry(5.0, 4000.0));
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 2000.0), Map.entry(2.0, 3500.0), Map.entry(5.0, 5000.0));

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final TalonFX leftKickerMotor;
  private final TalonFX rightKickerMotor;

  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withEnableFOC(false).withLimitReverseMotion(true);
  private double hubDistance = 0;
  private double feedDistance = 0;

  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;
  private double leftKickerMotorRpm = 0;
  private double rightKickerMotorRpm = 0;

  public Shooter(
      TalonFX leftMotor, TalonFX rightMotor, TalonFX leftKickerMotor, TalonFX rightKickerMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    var leftConfigs =
        new TalonFXConfiguration()
            // TODO: Get sensor to mechanism ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));
    var rightConfigs =
        new TalonFXConfiguration()
            // TODO: Get sensor to mechanism ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));
    var leftKickerConfigs =
        new TalonFXConfiguration()
            // TODO: Get sensor to mechanism ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));
    var rightKickerConfigs =
        new TalonFXConfiguration()
            // TODO: Get sensor to mechanism ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(100.0)
                    .withMotionMagicAcceleration(20.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));

    leftMotor.getConfigurator().apply(leftConfigs);
    rightMotor.getConfigurator().apply(rightConfigs);
    leftKickerMotor.getConfigurator().apply(leftKickerConfigs);
    rightKickerMotor.getConfigurator().apply(rightKickerConfigs);

    TunablePid.register("Shooter/LeftShooter", leftMotor, leftConfigs);
    TunablePid.register("Shooter/RightShooter", rightMotor, rightConfigs);
    TunablePid.register("Shooter/LeftKicker", leftKickerMotor, leftKickerConfigs);
    TunablePid.register("Shooter/RightKicker", rightKickerMotor, rightKickerConfigs);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.leftKickerMotor = leftKickerMotor;
    this.rightKickerMotor = rightKickerMotor;
  }

  public void setHubDistance(double distance) {
    this.hubDistance = distance;
  }

  public void setFeedDistance(double distance) {
    this.feedDistance = distance;
  }

  public void scoreRequest() {
    setStateFromRequest(ShooterState.SCORE);
  }

  public void feedRequest() {
    setStateFromRequest(ShooterState.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  @Override
  protected void whileInState(ShooterState state) {
    DogLog.log("Shooter/Left/RPM", leftMotorRpm);
    DogLog.log("Shooter/Right/RPM", rightMotorRpm);
    DogLog.log("Shooter/LeftKicker/RPM", leftKickerMotorRpm);
    DogLog.log("Shooter/RightKicker/RPM", rightKickerMotorRpm);
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());

    switch (state) {
      case SCORE -> {
        leftMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
        rightMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
        leftKickerMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
        rightKickerMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
      }
      case FEEDING -> {
        leftMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
        rightMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
        leftKickerMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
        rightKickerMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
      }
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
        leftKickerMotor.disable();
        rightKickerMotor.disable();
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(MAX_SAFE_RPM, DISTANCE_TO_SCORE_RPM.get(hubDistance));
    feedingRpm = Math.min(MAX_SAFE_RPM, DISTANCE_TO_FEEDING_RPM.get(feedDistance));

    leftMotorRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    rightMotorRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;
    leftKickerMotorRpm = leftKickerMotor.getVelocity().getValueAsDouble() * 60.0;
    rightKickerMotorRpm = rightKickerMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case SCORE ->
          MathUtil.isNear(leftMotorRpm, shootingRpm, 50)
              && MathUtil.isNear(rightMotorRpm, shootingRpm, 50)
              && MathUtil.isNear(leftKickerMotorRpm, shootingRpm, 50)
              && MathUtil.isNear(rightKickerMotorRpm, shootingRpm, 50);
      case FEEDING ->
          MathUtil.isNear(leftMotorRpm, feedingRpm, 100)
              && MathUtil.isNear(rightMotorRpm, feedingRpm, 100)
              && MathUtil.isNear(leftKickerMotorRpm, feedingRpm, 100)
              && MathUtil.isNear(rightKickerMotorRpm, feedingRpm, 100);
    };
  }

  @Override
  public void simulationPeriodic() {
    var shooterSimulation =
        SimKit.velocityMechanism(
            "shooter",
            (mechanism) ->
                mechanism
                    .addMotor(leftMotor)
                    .addMotor(rightMotor)
                    .addMotor(leftKickerMotor)
                    .addMotor(rightKickerMotor)
                    .withMinVelocity(0));

    shooterSimulation.update();
  }
}
