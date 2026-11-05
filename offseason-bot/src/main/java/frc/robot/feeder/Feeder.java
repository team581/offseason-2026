package frc.robot.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RollerUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final TalonFX motor;

  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> supplyVoltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Current> statorCurrentSignal;

  private double statorCurrent = 0.0;
  private final Debouncer emptyDebouncer = new Debouncer(0.5);
  private boolean hasFinishedShooting = false;

  private double surfaceSpeedMetersPerSecond = 0.0;

  public Feeder(TalonFX motor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    motor.getConfigurator().apply(FeederConfig.MOTOR_CONFIG);
    this.motor = motor;

    velocitySignal = motor.getRotorVelocity(false);
    statorCurrentSignal = motor.getStatorCurrent(false);
    Signals.forDevice(motor).addSignals(velocitySignal, statorCurrentSignal);
    supplyVoltageSignal = motor.getSupplyVoltage(false);
    Signals.forDevice(motor).addSignals(supplyVoltageSignal);
    supplyCurrentSignal = motor.getSupplyCurrent(false);
    Signals.forDevice(motor).addSignals(supplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    var limits = FeederConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit);
    motor.getConfigurator().apply(limits);
    DogLog.log("Feeder/Motor/SupplyLimitAmps", limits.SupplyCurrentLimit);
    DogLog.log("Feeder/Motor/StatorLimitAmps", limits.StatorCurrentLimit);
  }

  public void ballFillingRequest() {
    setStateFromRequest(FeederState.BALL_FILLING);
  }

  public void ejectRequest() {
    setStateFromRequest(FeederState.EJECT);
  }

  public void feedRequest() {
    setStateFromRequest(FeederState.FEED);
  }

  public double getStatorCurrent() {
    return statorCurrent;
  }

  public boolean hasFinishedShooting() {
    return hasFinishedShooting;
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FeederState.INTAKING);
  }

  public void scoreRequest() {
    setStateFromRequest(FeederState.SCORE);
  }

  @Override
  protected void afterTransition(FeederState newState) {
    emptyDebouncer.calculate(false);
    hasFinishedShooting = false;
    switch (newState) {
      case IDLE -> {
        motor.setControl(neutralRequest);
      }
      default -> {
        motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Feeder/Motor/SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
    DogLog.log("Feeder/Motor/SupplyCurrent", supplyCurrentSignal.getValueAsDouble());

    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double velocityRps = velocitySignal.getValueAsDouble();
    surfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            velocityRps, FeederConfig.ROLLER_DIAMETER_METERS, FeederConfig.MOTOR_TO_ROLLER_RATIO);
    DogLog.log("Feeder/Motor/VelocityRPM", velocityRps * 60.0);
    DogLog.log("Feeder/Motor/SurfaceSpeedMetersPerSecond", surfaceSpeedMetersPerSecond);

    statorCurrent = statorCurrentSignal.getValueAsDouble();
    emptyDebouncer.setDebounceTime(FeederConfig.EMPTY_DEBOUNCE_SECONDS.get());
    hasFinishedShooting =
        emptyDebouncer.calculate(
            (getState() == FeederState.SCORE || getState() == FeederState.FEED)
                && statorCurrent < FeederConfig.FUEL_CURRENT_THRESHOLD_AMPS.get());
    DogLog.log("Feeder/StatorCurrentAmps", statorCurrent);
    DogLog.log("Feeder/HasFinishedShooting", hasFinishedShooting);
  }
}
