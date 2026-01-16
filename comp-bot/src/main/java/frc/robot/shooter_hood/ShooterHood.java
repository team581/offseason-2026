package frc.robot.shooter_hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;

public class ShooterHood extends StateMachineSubsystem<ShooterHoodState> {
  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withEnableFOC(false);
  private double distance = 0;

  public ShooterHood(TalonFX motor) {
    super(SubsystemPriority.SHOOTER_HOOD, ShooterHoodState.UNHOMED);

    var config =
        new TalonFXConfiguration()
            // TODO: Figure out gearing ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));
    motor.getConfigurator().apply(config);

    this.motor = motor;

    TunablePid.register("ShooterHood", motor, config);
  }

  public void scoreRequest(double distance) {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> {
        this.distance = distance;
        setStateFromRequest(ShooterHoodState.SCORING);
      }
    }
  }

  public void feedRequest(double distance) {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> {
        this.distance = distance;
        setStateFromRequest(ShooterHoodState.FEEDING);
      }
    }
  }

  public void idleRequest() {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> {
        setStateFromRequest(ShooterHoodState.IDLE);
      }
    }
  }

  public void homingRequest() {
    setStateFromRequest(ShooterHoodState.HOMING);
  }

  @Override
  protected ShooterHoodState getNextState(ShooterHoodState currentState) {
    return switch (currentState) {
      case HOMING -> {
        // While we are homing, check the stator current draw of the motor
        // Once the current exceeds a threshold (20A), reset the encoder position to 0 and switch to
        // IDLE
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(ShooterHoodState newState) {
    switch (newState) {
      case HOMING -> {
        motor.setVoltage(0);
      }
      case UNHOMED -> {
        motor.disable();
      }
      default -> {}
    }
  }

  @Override
  protected void whileInState(ShooterHoodState state) {
    switch (state) {
      case SCORING -> {
        // TODO: Fix this setpoint
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(0)));
      }
      case FEEDING -> {
        // TODO: Fix this setpoint
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(0)));
      }
      case IDLE -> {
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(0)));
      }
      default -> {}
    }
  }
}
