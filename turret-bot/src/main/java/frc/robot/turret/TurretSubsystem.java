package frc.robot.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class TurretSubsystem extends StateMachineSubsystem<TurretState> {
  private final TalonFX motor;
  private final LocalizationSubsystem localization;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double hubAimAngle = 0.0;
  private static final double MIN_ANGLE = 0.0;
  private static final double MAX_ANGLE = 270.0;
  private static final double MANUAL_AIM_ANGLE = 50.0;
  private static final double HOMING_VOLTAGE = 1.0;
  private static final double HOMING_CURRENT_THRESHOLD =
      1.5; // Half of compbot 2025 deploy threshold
  private static final double HOMING_END_POSITION = 0.0;
  private static final double TOLERANCE = 1.0;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);
  private final DoubleSubscriber SHOOT_ON_THE_MOVE_LOOKAHEAD =
      DogLog.tunable("ShootOnTheMoveLookahead", 0.0);
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  public TurretSubsystem(TalonFX motor, LocalizationSubsystem localization) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);
var configs = new TalonFXConfiguration()
                .withFeedback(
                    new FeedbackConfigs()
                        .withSensorToMechanismRatio((280.0 / 12.0) * (40.0 / 12.0)))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(30)
                        .withStatorCurrentLimit(30))
                .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKG(0.0));
    motor
        .getConfigurator()
        .apply(configs
            );

                TunablePid.of("Deploy", motor, configs);

    this.motor = motor;
    this.localization = localization;
  }

  @Override
  protected void collectInputs() {
    var target = new Pose2d(11.91, 4.035, Rotation2d.kZero);
    var current = localization.getLookaheadPose(SHOOT_ON_THE_MOVE_LOOKAHEAD.get());
    var angle =
        Units.radiansToDegrees(
            Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
    var imuAngle = current.getRotation().getDegrees();
    hubAimAngle = angle - imuAngle;

    switch (getState()) {
      case UNHOMED, HOMING -> {
        rawCurrent = motor.getStatorCurrent().getValueAsDouble();
        filteredCurrent = currentFilter.calculate(rawCurrent);
      }
      case HUB_AIM -> goalAngle = hubAimAngle;
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
        motor.setPosition(0.0);
      }
      case HUB_AIM -> {
        motor.setControl(
            positionRequest.withPosition(Units.degreesToRotations(clamp(hubAimAngle))));
      }
      case MANUAL_AIM -> {
        motor.setControl(
            positionRequest.withPosition(Units.degreesToRotations(clamp(MANUAL_AIM_ANGLE))));
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
    var newTurretAngle = MathUtil.inputModulus(turretAngle, MIN_ANGLE, MAX_ANGLE);
    return MathUtil.clamp(newTurretAngle, MIN_ANGLE, MAX_ANGLE);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
if (RobotBase.isSimulation()) {
  simulationPeriodic();
}
    switch (getState()) {
      case HUB_AIM, MANUAL_AIM -> {
        afterTransition(getState());
        DogLog.clearFault("Turret is not homed");
      }
      case UNHOMED -> {
        DogLog.logFault("Turret is not homed", AlertType.kError);
      }
      default -> {
        DogLog.clearFault("Turret is not homed");
      }
    }
  }

  public void manualAimRequest() {
    setState(TurretState.MANUAL_AIM);
  }

  public void hubAimRequest() {
    setState(TurretState.HUB_AIM);
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


  public void simulationPeriodic() {
    var turretSimulation =
        SimKit.positionMechanism(
            "turret",
            (mechanism) ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(
                        Units.degreesToRotations(MIN_ANGLE))
                    .withMaxPosition(
                        Units.degreesToRotations(MAX_ANGLE)));

    if (getState() == TurretState.UNHOMED || getState() == TurretState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(TurretState.IDLE);
    }

    turretSimulation.update();
  }
}
