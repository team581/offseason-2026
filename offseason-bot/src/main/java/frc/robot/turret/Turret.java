package frc.robot.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.AimParameterUtil.AimingParameters;
import frc.robot.util.scheduling.SubsystemPriority;

public class Turret extends StateMachineSubsystem<TurretState> implements PowerManaged {
  private static double clamp(double wantedAngle) {
    return MathUtil.clamp(wantedAngle, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  static boolean isAtGoal(
      TurretState state, double setpoint, double currentAngle, double tolerance) {
    return switch (state) {
      case SCORE, FEED -> MathUtil.isNear(setpoint, currentAngle, tolerance);
      default -> false;
    };
  }

  static boolean isAtGoal(
      TurretState state,
      double setpoint,
      double currentAngle,
      double tolerance,
      double upcomingAngle) {
    return switch (state) {
      case SCORE, FEED -> {
        var potentialSetpoint = TurretCalculator.getOptimalAngle(upcomingAngle, currentAngle);
        if (!MathUtil.isNear(potentialSetpoint, setpoint, 90)) {
          yield false;
        }
        yield MathUtil.isNear(setpoint, currentAngle, tolerance, -180, 180);
      }
      default -> false;
    };
  }

  static boolean isAtGoal(
      TurretState state,
      TurretState expectedState,
      double currentAngle,
      AimingParameters aimingParameters) {
    if (!aimingParameters.solutionValid() || state != expectedState) {
      return false;
    }
    var targetSetpoint =
        clamp(TurretCalculator.getOptimalAngle(aimingParameters.turretAngle(), currentAngle));
    return MathUtil.isNear(
            aimingParameters.turretAngle(),
            currentAngle,
            aimingParameters.turretTolerance(),
            -180.0,
            180.0)
        && isAtGoal(
            state,
            targetSetpoint,
            currentAngle,
            aimingParameters.turretTolerance(),
            aimingParameters.upcomingTurretAngle());
  }

  static boolean isAtGoal(
      TurretState state,
      TurretState expectedState,
      double setpoint,
      double currentAngle,
      double tolerance) {
    if (state != expectedState) {
      return false;
    }
    return isAtGoal(state, setpoint, currentAngle, tolerance);
  }

  static boolean isAtGoal(
      TurretState state,
      TurretState expectedState,
      double setpoint,
      double currentAngle,
      double tolerance,
      double upcomingAngle) {
    if (state != expectedState) {
      return false;
    }
    return isAtGoal(state, setpoint, currentAngle, tolerance, upcomingAngle);
  }

  private final TalonFX motor;
  private final TurretHoming homing = new TurretHoming();
  private double currentAngle = 0.0;
  private double goalAngle = 0.0;
  private double setpoint = 0.0;
  private double velocity = 0.0;

  private double voltage = 0.0;

  private double statorCurrent = 0.0;
  private double feedForward = 0.0;

  // Send each same-loop target immediately instead of waiting for periodic CAN transmission.
  private final PositionVoltage positionRequest =
      new PositionVoltage(0.0).withEnableFOC(false).withUpdateFreqHz(0.0);

  private final NeutralOut neutralRequest = new NeutralOut();

  public Turret(TalonFX motor) {
    super(SubsystemPriority.TURRET, TurretState.HOMING);

    motor.getConfigurator().apply(TurretConfig.MOTOR_CONFIG);

    TunablePid.register("Turret", motor, TurretConfig.MOTOR_CONFIG);

    this.motor = motor;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(TurretConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }

  public boolean atGoal(AimingParameters aimingParameters) {
    return isAtGoal(getState(), getState(), currentAngle, aimingParameters);
  }

  public boolean atGoal(double tolerance) {
    return isAtGoal(getState(), setpoint, currentAngle, tolerance);
  }

  public boolean atGoal(double tolerance, double upcomingAngle) {
    return isAtGoal(getState(), setpoint, currentAngle, tolerance, upcomingAngle);
  }

  public boolean atGoal(TurretState expectedState, AimingParameters aimingParameters) {
    return isAtGoal(getState(), expectedState, currentAngle, aimingParameters);
  }

  public boolean atGoal(TurretState expectedState, double tolerance) {
    return isAtGoal(getState(), expectedState, setpoint, currentAngle, tolerance);
  }

  public boolean atGoal(TurretState expectedState, double tolerance, double upcomingAngle) {
    return isAtGoal(getState(), expectedState, setpoint, currentAngle, tolerance, upcomingAngle);
  }

  public void feedRequest(AimingParameters parameters) {
    feedRequest(parameters.turretAngle(), parameters.turretFeedForwardRadians());
  }

  public void feedRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    this.goalAngle = goalAngle;
    setState(TurretState.FEED);
  }

  public double getAngle() {
    return currentAngle;
  }

  public double getVelocity() {
    return velocity;
  }

  public void idleFeedRequest(AimingParameters parameters) {
    idleFeedRequest(parameters.turretAngle(), parameters.turretFeedForwardRadians());
  }

  public void idleFeedRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_FEED);
  }

  public void idleScoreRequest(AimingParameters parameters) {
    idleScoreRequest(parameters.turretAngle(), parameters.turretFeedForwardRadians());
  }

  public void idleScoreRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    this.goalAngle = goalAngle;
    setState(TurretState.IDLE_SCORE);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    switch (getState()) {
      case SCORE, FEED -> afterTransition(getState());
      default -> {}
    }

    // Homing completes on enable, so the turret is always unhomed while disabled.
    if (getState() == TurretState.HOMING && DriverStation.isEnabled()) {
      DogLog.logFault("Turret is not homed", AlertType.kError);
    } else {
      DogLog.clearFault("Turret is not homed");
    }

    if (getState() != TurretState.HOMING
        && DriverStation.isDisabled()
        && DriverStation.isAutonomous()
        && !MathUtil.isNear(setpoint, currentAngle, 10.0)) {
      DogLog.logFault("Turret is misaligned", AlertType.kWarning);
    } else {
      DogLog.clearFault("Turret is misaligned");
    }
  }

  public void scoreRequest(AimingParameters parameters) {
    scoreRequest(parameters.turretAngle(), parameters.turretFeedForwardRadians());
  }

  public void scoreRequest(double goalAngle, double feedForward) {
    this.feedForward = feedForward;
    this.goalAngle = goalAngle;
    setState(TurretState.SCORE);
  }

  public void setState(TurretState newState) {
    switch (getState()) {
      case HOMING -> {
        if (RobotBase.isSimulation()) {
          setStateFromRequest(newState);
        }
      }
      default -> setStateFromRequest(newState);
    }
  }

  @Override
  public void simulationPeriodic() {
    var turretSimulation =
        SimKit.positionMechanism(
            "turret",
            mechanism ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(Units.degreesToRotations(TurretConfig.MIN_ANGLE))
                    .withMaxPosition(Units.degreesToRotations(TurretConfig.MAX_ANGLE)));

    if (getState() == TurretState.HOMING) {
      turretSimulation.seedPosition(Units.degreesToRotations(TurretConfig.HOMING_END_POSITION));
      currentAngle = TurretConfig.HOMING_END_POSITION;
      setStateFromRequest(TurretState.SCORE);
    }

    turretSimulation.update();
  }

  public void stuckRequest() {
    if (getState() != TurretState.HOMING) {
      setStateFromRequest(TurretState.STUCK);
    }
  }

  private double getAngleError() {
    return MathUtil.inputModulus(setpoint - currentAngle, -180.0, 180.0);
  }

  private double getFeedForward() {
    if (MathUtil.isNear(TurretConfig.MAX_ANGLE, currentAngle, 3)
        || MathUtil.isNear(TurretConfig.MIN_ANGLE, currentAngle, 3)) {
      return 0.0;
    }
    return feedForward;
  }

  private void updateSetpoint() {
    setpoint =
        switch (getState()) {
          case HOMING -> 0.0;
          case SCORE, FEED, STUCK ->
              clamp(TurretCalculator.getOptimalAngle(goalAngle, currentAngle));
          case IDLE_SCORE, IDLE_FEED ->
              clamp(TurretCalculator.getSmartUnwrapAngle(goalAngle, currentAngle));
        };
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());

    velocity = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    voltage = motor.getMotorVoltage().getValueAsDouble();
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();

    // Predict the turret's current angle for diagnostics.
    double latencyCompensatedAngle =
        Units.rotationsToDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                motor.getPosition(), motor.getVelocity()));

    DogLog.log("Turret/Angle", currentAngle);
    DogLog.log("Turret/VelocityDegreesPerSecond", velocity);
    DogLog.log("Turret/Motor/LatencyCompensatedAngle", latencyCompensatedAngle);
    DogLog.log("Turret/Motor/RotorPosition", motor.getRotorPosition().getValueAsDouble());
  }

  @Override
  protected TurretState getNextState(TurretState currentState) {
    switch (currentState) {
      case HOMING -> {
        if (RobotBase.isReal() && updateHoming(DriverStation.isDisabled())) {
          return TurretState.SCORE;
        }
        return currentState;
      }
      default -> {
        return currentState;
      }
    }
  }

  @Override
  protected void whileInState(TurretState currentState) {
    // RobotManager submits the current goal during its action phase. Derive the setpoint here so
    // the command sent to the motor uses that same-loop goal instead of the previous loop's one.
    updateSetpoint();

    switch (currentState) {
      case HOMING -> motor.setControl(neutralRequest);
      case STUCK -> {
        motor.setControl(
            positionRequest
                .withPosition(Units.degreesToRotations(clamp(TurretConfig.FAUX_DUMPER_ANGLE)))
                .withVelocity(0.0));
      }
      case SCORE, FEED ->
          motor.setControl(
              positionRequest
                  .withPosition(Units.degreesToRotations(clamp(setpoint)))
                  .withVelocity(Units.radiansToRotations(getFeedForward())));
      case IDLE_SCORE, IDLE_FEED ->
          motor.setControl(
              positionRequest
                  .withPosition(Units.degreesToRotations(clamp(setpoint)))
                  .withVelocity(Units.radiansToRotations(getFeedForward())));
      default -> {}
    }

    DogLog.log("Turret/StatorCurrent", statorCurrent);
    DogLog.log("Turret/Voltage", voltage);
    DogLog.log("Turret/Setpoint", setpoint);
    DogLog.log("Turret/FeedForwardRadiansPerSecond", getFeedForward());
    DogLog.log("Turret/AngleErrorDegrees", getAngleError());
  }

  /** Runs one homing step against the motor, returning true once homed after enable. */
  boolean updateHoming(boolean disabled) {
    var rotorPosition = motor.getRotorPosition();
    var position = motor.getPosition();
    boolean inputsValid =
        motor.isAlive()
            && motor.isConnected()
            && rotorPosition.getStatus().isOK()
            && position.getStatus().isOK();
    DogLog.log("Turret/Homing/InputsValid", inputsValid);
    DogLog.log(
        "Turret/Homing/HomedAngle",
        Units.rotationsToDegrees(
            TurretCalculator.calculateHomedPosition(rotorPosition.getValueAsDouble())));
    return homing.update(
        disabled,
        inputsValid,
        rotorPosition.getValueAsDouble(),
        Units.rotationsToDegrees(position.getValueAsDouble()),
        homedPosition -> {
          var status = motor.setPosition(homedPosition);
          DogLog.log("Turret/Homing/SetPositionStatus", status.toString());
          if (!status.isOK()) {
            return false;
          }
          currentAngle = Units.rotationsToDegrees(homedPosition);
          return true;
        });
  }
}
