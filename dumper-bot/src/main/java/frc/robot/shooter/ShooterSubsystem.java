package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;

public class ShooterSubsystem extends StateMachineSubsystem<ShooterState> {
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 1000.0), Map.entry(2.0, 2500.0), Map.entry(5.0, 4000.0));
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 2000.0), Map.entry(2.0, 3500.0), Map.entry(5.0, 5000.0));

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private double distance = 0;
  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;

  public ShooterSubsystem(TalonFX leftMotor, TalonFX rightMotor) {

    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);
    leftMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0)));
    rightMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0)));
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void scoreRequest(double distance) {
    this.distance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void feedRequest(double distance) {
    this.distance = distance;
    setStateFromRequest(ShooterState.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  @Override
  protected void whileInState(ShooterState state) {
    DogLog.log("Shooter/State", getState());
    DogLog.log("Shooter/LeftMotorRPM", leftMotorRpm);
    DogLog.log("Shooter/RightMotorRPM", rightMotorRpm);
    DogLog.log("Shooter/ShootingRPM", shootingRpm);
    DogLog.log("Shooter/FeedingRPM", feedingRpm);
    DogLog.log("Shooter/AtGoal", atGoal());

    switch (state) {
      case SCORE -> {
        leftMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
        rightMotor.setControl(velocityRequest.withVelocity(shootingRpm / 60.0));
      }
      case FEEDING -> {
        leftMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
        rightMotor.setControl(velocityRequest.withVelocity(feedingRpm / 60.0));
      }
      case IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }
    }
  }

  @Override
  protected void collectInputs() {
    shootingRpm = Math.min(4000, DISTANCE_TO_SCORE_RPM.get(distance));
    feedingRpm = Math.min(5000, DISTANCE_TO_FEEDING_RPM.get(distance));

    leftMotorRpm = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    rightMotorRpm = rightMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case SCORE ->
          MathUtil.isNear(leftMotorRpm, shootingRpm, 50)
              && MathUtil.isNear(rightMotorRpm, shootingRpm, 50);
      case FEEDING ->
          MathUtil.isNear(leftMotorRpm, feedingRpm, 100)
              && MathUtil.isNear(rightMotorRpm, feedingRpm, 100);
    };
  }
}
