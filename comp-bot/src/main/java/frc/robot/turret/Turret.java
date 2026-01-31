package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  private final LinearFilter currentFilter = LinearFilter.movingAverage(7);
  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  private final Vision vision;

  public Turret(TalonFX motor, Vision vision) {
    super(SubsystemPriority.TURRET, TurretState.UNHOMED);
    this.vision = vision;
    motor.getConfigurator().apply(TurretConfig.MOTOR_CONFIG);

    TunablePid.register("Turret", motor, TurretConfig.MOTOR_CONFIG);

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
        motor.setVoltage(TurretConfig.HOMING_VOLTAGE.get());
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

      default -> {}
    }
  }

  @Override
  protected TurretState getNextState(TurretState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (filteredCurrent > TurretConfig.HOMING_CURRENT_THRESHOLD.get()) {
          motor.setPosition(Units.degreesToRotations(TurretConfig.HOMING_END_POSITION));
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
    return MathUtil.clamp(newTurretAngle, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public boolean goalOutOfBounds() {
    return goalAngle > (TurretConfig.MAX_ANGLE - TurretConfig.OUT_OF_BOUNDS_THRESHOLD)
        || goalAngle < (TurretConfig.MIN_ANGLE + TurretConfig.OUT_OF_BOUNDS_THRESHOLD);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    switch (getState()) {
      case HUB_AIM, FEED_AIM -> {
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

  public void idleRequest() {
    setState(TurretState.IDLE);
  }

  public void setHubAimAngle(double angle) {
    hubAimAngle = angle;
  }

  public void setFeedAimAngle(double angle) {
    feedAimAngle = angle;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING, IDLE -> false;
      default -> MathUtil.isNear(clamp(goalAngle), currentAngle, TurretConfig.TOLERANCE);
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
                    .withMinPosition(Units.degreesToRotations(TurretConfig.MIN_ANGLE))
                    .withMaxPosition(Units.degreesToRotations(TurretConfig.MAX_ANGLE)));

    if (getState() == TurretState.UNHOMED || getState() == TurretState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(TurretState.IDLE);
    }

    turretSimulation.update();
  }
}
