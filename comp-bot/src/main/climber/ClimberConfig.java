package frc.robot.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ClimberConfig {
  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimit(30))
          .withSlot0(new Slot0Configs().withKP(150.0).withKV(0.0).withKG(0.0));
}
