package frc.robot.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends StateMachineSubsystem<ClimberState> {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  public Climber(TalonFX motor) {
    this.motor = motor;

    // TODO: This should be in ClimberConfig
    var configs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimit(30))
            .withSlot0(new Slot0Configs().withKP(150.0).withKV(0.0).withKG(0.0));
    motor.getConfigurator().apply(configs);

    TunablePid.register("Climber", this.motor, configs);
  }

  public void l1LineupRequest() {
    setStateFromRequest(ClimberState.L1_LINEUP);
  }

  public void l1_hangingRequest() {
    setStateFromRequest(ClimberState.L1_HANGING);
  }

  public void l2_hangingRequest() {
    setStateFromRequest(ClimberState.L2_HANGING);
  }

  public void l2_lineupRequest() {
    setStateFromRequest(ClimberState.L2_LINEUP);
  }

  public void l3_hangingRequest() {
    setStateFromRequest(ClimberState.L3_HANGING);
  }

  public void l3_lineupRequest() {
    setStateFromRequest(ClimberState.L3_LINEUP);
  }

  public void stowedRequest() {
    setStateFromRequest(ClimberState.STOWED);
  }
}
