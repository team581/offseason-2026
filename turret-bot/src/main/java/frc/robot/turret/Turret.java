package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class Turret extends StateMachineSubsystem<TurretState> {
  private final TalonFX motor;
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double hubAimAngle = 0.0;
  private double feedAimAngle = 0.0;
  private double tagAimAngle = 0.0;

  private static final double MIN_ANGLE = -149.105;
  private static final double MAX_ANGLE = 149.105;
  private static final double OUT_OF_BOUNDS_THRESHOLD = 1.0;
  private static final double MANUAL_AIM_ANGLE = 0.0;
  private static final DoubleSubscriber HOMING_VOLTAGE =
      DogLog.tunable("TurretHomingVoltage", -2.0);
  private static final DoubleSubscriber HOMING_CURRENT_THRESHOLD =
      DogLog.tunable("TurretCurrentThreshold", 3.0);
  private static final double HOMING_END_POSITION = MIN_ANGLE;
  private static final double TOLERANCE = 1.0;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  private final Vision vision;

  public Turret(TalonFX motor, Vision vision) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);
    this.vision = vision;
    var configs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio((280.0 / 12.0) * (40.0 / 12.0)))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimit(30))
            .withSlot0(new Slot0Configs().withKP(150.0).withKV(0.0).withKG(0.0));
    motor.getConfigurator().apply(configs);

    TunablePid.register("Turret", motor, configs);

    this.motor = motor;
  }

  @Override
  protected void collectInputs() {

    switch (getState()) {
      case UNHOMED, HOMING -> {
        rawCurrent = motor.getStatorCurrent().getValueAsDouble();
        filteredCurrent = currentFilter.calculate(rawCurrent);
      }
      case HUB_AIM -> goalAngle = hubAimAngle;
      case FEED_AIM -> goalAngle = feedAimAngle;
      case TAG_AIM -> goalAngle = tagAimAngle;
      case LOCK_FORWARD -> goalAngle = MANUAL_AIM_ANGLE;
      case IDLE -> {}
    }

    currentAngle =
        MathHelpers.angleModulus(Units.rotationsToDegrees(motor.getPosition().getValueAsDouble()));
    DogLog.log("Turret/Angle", currentAngle);

    // Predict the turret's current angle to account for sensor latency
    double latencyCompensatedAngle =
        Units.rotationsToDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                motor.getPosition(), motor.getVelocity()));
    DogLog.log("Turret/LatencyCompensatedAngle", latencyCompensatedAngle);

    // Add the predicted angle to the vision buffer at the current timestamp
    vision.addTurretObservation(
        Timer.getFPGATimestamp(),
        Rotation2d.fromDegrees(latencyCompensatedAngle),
        getVelocityDegreesPerSecond());
  }

  @Override
  protected void afterTransition(TurretState newState) {
    switch (newState) {
      case UNHOMED -> {
        motor.disable();
      }
      case HOMING -> {
        motor.setVoltage(HOMING_VOLTAGE.get());
      }
      case IDLE -> {
        motor.setControl(positionRequest.withPosition(Units.degreesToRotations(clamp(0.0))));
      }
      case HUB_AIM -> {
        motor.setControl(
            positionRequest.withPosition(Units.degreesToRotations(clamp(hubAimAngle))));
      }
      case FEED_AIM -> {
        motor.setControl(
            positionRequest.withPosition(Units.degreesToRotations(clamp(feedAimAngle))));
      }
      case TAG_AIM -> {
        motor.setControl(
            positionRequest.withPosition(Units.degreesToRotations(clamp(tagAimAngle))));
      }
      case LOCK_FORWARD -> {
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
        if (filteredCurrent > HOMING_CURRENT_THRESHOLD.get()) {
          motor.setPosition(Units.degreesToRotations(HOMING_END_POSITION));
          motor.disable();
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
    var newTurretAngle = MathHelpers.angleModulus(turretAngle);
    return MathUtil.clamp(newTurretAngle, MIN_ANGLE, MAX_ANGLE);
  }

  public boolean goalOutOfBounds() {
    return goalAngle > (MAX_ANGLE - OUT_OF_BOUNDS_THRESHOLD)
        || goalAngle < (MIN_ANGLE + OUT_OF_BOUNDS_THRESHOLD);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    switch (getState()) {
      case HUB_AIM, FEED_AIM, TAG_AIM, LOCK_FORWARD -> {
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

  public void hubAimRequest() {
    setState(TurretState.HUB_AIM);
  }

  public void feedAimRequest() {
    setState(TurretState.FEED_AIM);
  }

  public void tagAimRequest() {
    setState(TurretState.TAG_AIM);
  }

  public void lockForwardRequest() {
    setState(TurretState.LOCK_FORWARD);
  }

  public void idleRequest() {
    setState(TurretState.IDLE);
  }

  public void setHubAimAngle(double angle) {
    hubAimAngle = angle;
  }

  public void setFeedAimAngle(double angle) {
    feedAimAngle = angle;
  }

  public void setTagAimAngle(double angle) {
    tagAimAngle = angle;
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

  public double getVelocityDegreesPerSecond() {
    return Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
  }

  public void homeRequest() {
    setState(TurretState.HOMING);
  }

  @Override
  public void simulationPeriodic() {
    var turretSimulation =
        SimKit.positionMechanism(
            "turret",
            (mechanism) ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(Units.degreesToRotations(MIN_ANGLE))
                    .withMaxPosition(Units.degreesToRotations(MAX_ANGLE)));

    if (getState() == TurretState.UNHOMED || getState() == TurretState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(TurretState.IDLE);
    }

    turretSimulation.update();
  }
}
