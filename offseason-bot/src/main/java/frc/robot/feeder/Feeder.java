package frc.robot.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
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

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> topSupplyVoltageSignal;
  private final StatusSignal<Current> topSupplyCurrentSignal;
  private final StatusSignal<Voltage> bottomSupplyVoltageSignal;
  private final StatusSignal<Current> bottomSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> topVelocitySignal;
  private final StatusSignal<AngularVelocity> bottomVelocitySignal;
  private final StatusSignal<Current> topStatorCurrentSignal;
  private final StatusSignal<Current> bottomStatorCurrentSignal;

  private double averageCurrent = 0.0;
  private final Debouncer emptyDebouncer = new Debouncer(0.5);
  private boolean hasFinishedShooting = false;

  private double topSurfaceSpeedMetersPerSecond = 0.0;
  private double bottomSurfaceSpeedMetersPerSecond = 0.0;

  public Feeder(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    topMotor.getConfigurator().apply(FeederConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(FeederConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;

    topVelocitySignal = topMotor.getRotorVelocity(false);
    bottomVelocitySignal = bottomMotor.getRotorVelocity(false);
    topStatorCurrentSignal = topMotor.getStatorCurrent(false);
    bottomStatorCurrentSignal = bottomMotor.getStatorCurrent(false);
    Signals.forDevice(topMotor).addSignals(topVelocitySignal, topStatorCurrentSignal);
    Signals.forDevice(bottomMotor).addSignals(bottomVelocitySignal, bottomStatorCurrentSignal);
    topSupplyVoltageSignal = topMotor.getSupplyVoltage(false);
    Signals.forDevice(topMotor).addSignals(topSupplyVoltageSignal);
    topSupplyCurrentSignal = topMotor.getSupplyCurrent(false);
    Signals.forDevice(topMotor).addSignals(topSupplyCurrentSignal);
    bottomSupplyVoltageSignal = bottomMotor.getSupplyVoltage(false);
    Signals.forDevice(bottomMotor).addSignals(bottomSupplyVoltageSignal);
    bottomSupplyCurrentSignal = bottomMotor.getSupplyCurrent(false);
    Signals.forDevice(bottomMotor).addSignals(bottomSupplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    var topLimits =
        FeederConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit);
    topMotor.getConfigurator().apply(topLimits);
    DogLog.log("Feeder/Top/SupplyLimitAmps", topLimits.SupplyCurrentLimit);
    DogLog.log("Feeder/Top/StatorLimitAmps", topLimits.StatorCurrentLimit);
    var bottomLimits =
        FeederConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit);
    bottomMotor.getConfigurator().apply(bottomLimits);
    DogLog.log("Feeder/Bottom/SupplyLimitAmps", bottomLimits.SupplyCurrentLimit);
    DogLog.log("Feeder/Bottom/StatorLimitAmps", bottomLimits.StatorCurrentLimit);
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

  public double getAverageCurrent() {
    return averageCurrent;
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
        topMotor.setControl(neutralRequest);
        bottomMotor.setControl(neutralRequest);
      }
      default -> {
        topMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        bottomMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Feeder/Top/SupplyVoltage", topSupplyVoltageSignal.getValueAsDouble());
    DogLog.log("Feeder/Top/SupplyCurrent", topSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Feeder/Bottom/SupplyVoltage", bottomSupplyVoltageSignal.getValueAsDouble());
    DogLog.log("Feeder/Bottom/SupplyCurrent", bottomSupplyCurrentSignal.getValueAsDouble());

    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double topVelocityRps = topVelocitySignal.getValueAsDouble();
    topSurfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            topVelocityRps,
            FeederConfig.TOP_ROLLER_DIAMETER_METERS,
            FeederConfig.MOTOR_TO_ROLLER_RATIO);
    double bottomVelocityRps = bottomVelocitySignal.getValueAsDouble();
    bottomSurfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            bottomVelocityRps,
            FeederConfig.BOTTOM_ROLLER_DIAMETER_METERS,
            FeederConfig.MOTOR_TO_ROLLER_RATIO);

    DogLog.log("Feeder/Top/VelocityRPM", topVelocityRps * 60.0);
    DogLog.log("Feeder/Top/SurfaceSpeedMetersPerSecond", topSurfaceSpeedMetersPerSecond);
    DogLog.log("Feeder/Bottom/VelocityRPM", bottomVelocityRps * 60.0);
    DogLog.log("Feeder/Bottom/SurfaceSpeedMetersPerSecond", bottomSurfaceSpeedMetersPerSecond);

    averageCurrent =
        MathHelpers.average(
            topStatorCurrentSignal.getValueAsDouble(),
            bottomStatorCurrentSignal.getValueAsDouble());
    emptyDebouncer.setDebounceTime(FeederConfig.EMPTY_DEBOUNCE_SECONDS.get());
    hasFinishedShooting =
        emptyDebouncer.calculate(
            (getState() == FeederState.SCORE || getState() == FeederState.FEED)
                && averageCurrent < FeederConfig.FUEL_CURRENT_THRESHOLD_AMPS.get());
    DogLog.log("Feeder/AverageCurrentAmps", averageCurrent);
    DogLog.log("Feeder/HasFinishedShooting", hasFinishedShooting);
  }
}
