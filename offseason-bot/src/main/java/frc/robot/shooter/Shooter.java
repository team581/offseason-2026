package frc.robot.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.util.RollerUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> implements PowerManaged {
  private static double distanceToFeedingRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEEDING_RPM.get(distance);
  }

  private static double distanceToScoringRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_RPM.get(distance);
  }

  static boolean isAtGoal(
      ShooterState state, double currentRpm, double shootingRpm, double feedingRpm) {
    return switch (state) {
      case IDLE -> false;
      case PREPARE_SCORE -> MathUtil.isNear(currentRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE);
      case SCORE ->
          MathUtil.isNear(currentRpm, shootingRpm, ShooterConfig.RPM_TOLERANCE_ACTIVELY_SHOOTING);
      case PREPARE_FEED, FEED ->
          MathUtil.isNear(currentRpm, feedingRpm, ShooterConfig.RPM_TOLERANCE_FEEDING);
    };
  }

  static boolean isAtGoal(
      ShooterState state,
      ShooterState expectedState,
      double currentRpm,
      double shootingRpm,
      double feedingRpm) {
    if (state != expectedState) {
      return false;
    }
    return isAtGoal(state, currentRpm, shootingRpm, feedingRpm);
  }

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityVoltage voltageVelocityRequest =
      new VelocityVoltage(0).withLimitReverseMotion(true).withEnableFOC(false);
  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;
  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;

  private final StatusSignal<Voltage> rightMotorVoltageSignal;
  private final StatusSignal<Current> rightStatorCurrentSignal;

  private double leftSupplyCurrent = 0;
  private double rightSupplyCurrent = 0;

  private double scoreDistance = 0;
  private double feedDistance = 0;
  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;

  private double initialFeedForward = 0.0;
  private boolean hopperFull = false;

  private boolean atGoal = false;

  private boolean atGoalDebounced = false;

  private boolean turboMode = false;

  private boolean hasReachedGoal = false;

  // Debounce for delay between shots at 15 bps
  private Debouncer atGoalDebouncer = new Debouncer(1.0 / 15.0, DebounceType.kFalling);

  public Shooter(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

    leftMotor.getConfigurator().apply(ShooterConfig.LEFT_MOTOR_CONFIG);
    DogLog.log(
        "Shooter/Right/ConfigurationStatus",
        rightMotor.getConfigurator().apply(ShooterConfig.RIGHT_MOTOR_CONFIG).toString());
    DogLog.log("Shooter/Right/DeviceId", rightMotor.getDeviceID());
    DogLog.log("Shooter/Right/CanBus", rightMotor.getNetwork().getName());

    TunablePid.register("Shooter/Left", leftMotor, ShooterConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Shooter/Right", rightMotor, ShooterConfig.RIGHT_MOTOR_CONFIG);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    leftVelocitySignal = leftMotor.getVelocity(false);
    rightVelocitySignal = rightMotor.getVelocity(false);
    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);

    Signals.forDevice(leftMotor).addSignals(leftVelocitySignal, leftSupplyCurrentSignal);
    rightMotorVoltageSignal = rightMotor.getMotorVoltage(false);
    rightStatorCurrentSignal = rightMotor.getStatorCurrent(false);
    Signals.forDevice(rightMotor)
        .addSignals(
            rightVelocitySignal,
            rightSupplyCurrentSignal,
            rightMotorVoltageSignal,
            rightStatorCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            ShooterConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            ShooterConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public boolean atGoal() {
    return atGoal;
  }

  public boolean atGoal(ShooterState expectedState) {
    return getState() == expectedState && atGoal;
  }

  public boolean atGoalDebounced() {
    return atGoalDebounced;
  }

  public boolean atGoalDebounced(ShooterState expectedState) {
    return getState() == expectedState && atGoalDebounced;
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.FEED);
  }

  public double getFeedTimeOfFlight(double distance) {
    return ShooterConfig.getFeedTimeOfFlight(distance);
  }

  public double getScoreTimeOfFlight(double distance) {
    return ShooterConfig.getScoreTimeOfFlight(distance);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  public void prepareFeedRequest(double distance) {
    this.feedDistance = distance;
    setStateFromRequest(ShooterState.PREPARE_FEED);
  }

  public void prepareScoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.PREPARE_SCORE);
  }

  public void scoreRequest(double distance) {
    this.scoreDistance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void setTurboMode(boolean useTurboMode) {
    turboMode = useTurboMode;
  }

  @Override
  public void simulationPeriodic() {
    var shooterSimulation =
        SimKit.velocityMechanism(
            "shooter",
            mechanism ->
                mechanism
                    .addMotor(leftMotor, ChassisReference.Clockwise_Positive)
                    .addMotor(rightMotor, ChassisReference.CounterClockwise_Positive));

    shooterSimulation.update();
  }

  public void updateHopperState(boolean hopperFull) {
    this.hopperFull = hopperFull;
  }

  private boolean calculateAtGoal() {
    return isAtGoal(getState(), rightMotorRpm, shootingRpm, feedingRpm);
  }

  private void setVelocity(double rotationsPerSecond, double simulationFeedForward) {
    if (RobotBase.isSimulation()) {
      rightMotor.setControl(
          velocityRequest.withVelocity(rotationsPerSecond).withFeedForward(simulationFeedForward));
    } else {
      // Beta's gains are voltage-based. Do not apply the old torque-current boost as volts.
      rightMotor.setControl(voltageVelocityRequest.withVelocity(rotationsPerSecond));
    }
  }

  private void updateRpmTargets() {
    shootingRpm =
        MathUtil.clamp(distanceToScoringRpm(scoreDistance), 0.0, ShooterConfig.MAX_SAFE_RPM);
    feedingRpm =
        MathUtil.clamp(distanceToFeedingRpm(feedDistance), 0.0, ShooterConfig.MAX_SAFE_RPM);

    if (DSOptions.PIT_FUNCTIONALITY.getAsBoolean()) {
      shootingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
      feedingRpm = ShooterConfig.PIT_FUNCTIONALITY_RPM;
    }
  }

  @Override
  protected void afterTransition(ShooterState newState) {
    atGoal = false;
    atGoalDebounced = false;
    hasReachedGoal = false;
    atGoalDebouncer = new Debouncer(1.0 / 15.0, DebounceType.kFalling);
  }

  @Override
  protected void collectInputs() {
    updateRpmTargets();

    leftMotorRpm = leftVelocitySignal.getValueAsDouble() * 60.0;
    rightMotorRpm = rightVelocitySignal.getValueAsDouble() * 60.0;
    DogLog.log("Shooter/Right/VelocityStatus", rightVelocitySignal.getStatus().toString());
    DogLog.log("Shooter/Right/MotorVoltage", rightMotorVoltageSignal.getValueAsDouble());
    DogLog.log("Shooter/Right/StatorCurrent", rightStatorCurrentSignal.getValueAsDouble());

    leftSupplyCurrent = leftSupplyCurrentSignal.getValueAsDouble();
    rightSupplyCurrent = rightSupplyCurrentSignal.getValueAsDouble();

    atGoal = calculateAtGoal();
    hasReachedGoal |= atGoal;
    // A falling-edge debouncer starts true; it cannot grant readiness before spin-up.
    atGoalDebounced = atGoalDebouncer.calculate(atGoal) && hasReachedGoal;

    switch (getState()) {
      case SCORE, FEED -> {
        if (!timeout(0.7) && hopperFull) {
          initialFeedForward = ShooterConfig.FULL_HOPPER_INITIAL_FF.getAsDouble();
        } else if (!timeout(0.1)) {
          initialFeedForward = ShooterConfig.LOW_HOPPER_INITIAL_FFF.getAsDouble();
        } else {
          initialFeedForward = 0.0;
        }
      }
      default -> {
        initialFeedForward = 0.0;
      }
    }
  }

  @Override
  protected void whileInState(ShooterState state) {
    // RobotManager has now supplied this loop's compensated distance.
    updateRpmTargets();
    DogLog.log("Shooter/Left/RPM", leftMotorRpm);
    DogLog.log("Shooter/Right/RPM", rightMotorRpm);
    // Undo Phoenix feedback scaling to recover motor RPS before applying physical gearing.
    DogLog.log(
        "Shooter/LeftSurfaceSpeedMetersPerSecond",
        RollerUtil.surfaceSpeedMetersPerSecond(
            leftMotorRpm / 60.0 * ShooterConfig.LEFT_MOTOR_CONFIG.Feedback.SensorToMechanismRatio,
            ShooterConfig.ROLLER_DIAMETER_METERS,
            ShooterConfig.MOTOR_TO_ROLLER_RATIO));
    DogLog.log(
        "Shooter/RightSurfaceSpeedMetersPerSecond",
        RollerUtil.surfaceSpeedMetersPerSecond(
            rightMotorRpm / 60.0 * ShooterConfig.RIGHT_MOTOR_CONFIG.Feedback.SensorToMechanismRatio,
            ShooterConfig.ROLLER_DIAMETER_METERS,
            ShooterConfig.MOTOR_TO_ROLLER_RATIO));
    DogLog.log("Shooter/GoalShootingRPM", shootingRpm);
    DogLog.log("Shooter/GoalFeedingRPM", feedingRpm);
    DogLog.log("Shooter/InitialFeedForward", initialFeedForward);
    DogLog.log("Shooter/AtGoal", atGoal);

    DogLog.log("Shooter/Left/SupplyCurrent", leftSupplyCurrent);
    DogLog.log("Shooter/Right/SupplyCurrent", rightSupplyCurrent);

    switch (state) {
      case IDLE -> {
        var setpoint = ShooterConfig.IDLE_RPM / 60.0;
        setVelocity(setpoint, 0.0);
        DogLog.log("Shooter/RpmSetpoint", ShooterConfig.IDLE_RPM);
      }
      case PREPARE_SCORE, SCORE -> {
        var setpoint = shootingRpm / 60.0;
        setVelocity(setpoint, initialFeedForward);
        DogLog.log("Shooter/RpmSetpoint", shootingRpm);
      }
      case PREPARE_FEED, FEED -> {
        var setpoint = feedingRpm / 60.0;
        setVelocity(setpoint, initialFeedForward);
        DogLog.log("Shooter/RpmSetpoint", feedingRpm);
      }
    }
  }
}
