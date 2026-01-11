package frc.robot.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class TurretSubsystem extends StateMachineSubsystem<TurretState> {
  private final TalonFX motor;
  private final LocalizationSubsystem localization;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double autoAimAngle = 0.0;
  private static final double MIN_ANGLE = 0.0;
  private static final double MAX_ANGLE = 0.0;
  private final double MANUAL_AIM_ANGLE = 0.0;
  private final double HOMING_VOLTAGE = 0.0;
  private final double HOMING_CURRENT_THRESHOLD = 1.5; // Half of compbot 2025 deploy threshold
  private final double HOMING_END_POSITION = 0.0;
  private final double TOLERANCE = 0.0;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;
  private PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  public TurretSubsystem(TalonFX motor, LocalizationSubsystem localization) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);

    motor.getConfigurator().apply(
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio(60.0 / 1.0))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(30)
                  .withStatorCurrentLimit(30))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.0)
                  .withKV(0.0)
                  .withKG(0.0)));
    this.motor = motor;
    this.localization = localization;
  }

  @Override
  protected void collectInputs() {
    autoAimAngle = localization.getAutoAimAngle();
    switch (getState()) {
      case UNHOMED, HOMING -> {
        rawCurrent = motor.getStatorCurrent().getValueAsDouble();
        filteredCurrent = currentFilter.calculate(rawCurrent);
      }
      case AUTO_AIM -> goalAngle = autoAimAngle;
      case MANUAL_AIM -> goalAngle = MANUAL_AIM_ANGLE;
      case IDLE -> {}
    }

    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  @Override
  protected void afterTransition(TurretState newState) {
    switch (newState) {
      case UNHOMED -> {
        motor.disable();
      }
      case HOMING -> {
        motor.setVoltage(HOMING_VOLTAGE);
      }
      case IDLE -> {
        motor.disable();
      }
      case AUTO_AIM -> {
        motor.setControl(positionRequest.withPosition(Units.degreesToRotations(autoAimAngle)));
      }
      case MANUAL_AIM -> {
        motor.setControl(positionRequest.withPosition(Units.degreesToRotations(MANUAL_AIM_ANGLE)));
      }
      default -> {}
    }
  }

  @Override
  protected TurretState getNextState(TurretState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (filteredCurrent > HOMING_CURRENT_THRESHOLD) {
          motor.setPosition(HOMING_END_POSITION);
          yield TurretState.IDLE;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  public void setState(TurretState newState) {
    switch (getState()) {
      case HOMING -> {}
      case UNHOMED -> {
        if (newState == TurretState.HOMING) {
          setStateFromRequest(TurretState.HOMING);
        }
      }
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  private static double clamp(double turretAngle) {
    return MathUtil.clamp(turretAngle, MIN_ANGLE, MAX_ANGLE);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING, IDLE -> false;
      default -> MathUtil.isNear(clamp(goalAngle), currentAngle, TOLERANCE);
    };
  }

  public double getAngle() {
    return currentAngle;
  }
}
