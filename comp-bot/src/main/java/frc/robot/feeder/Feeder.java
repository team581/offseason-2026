package frc.robot.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> topVelocitySignal;
  private final StatusSignal<AngularVelocity> bottomVelocitySignal;
  private final StatusSignal<Current> topStatorCurrentSignal;
  private final StatusSignal<Current> bottomStatorCurrentSignal;
  private final List<BaseStatusSignal> allSignals;

  private double averageCurrent = 0.0;

  public Feeder(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    topMotor.getConfigurator().apply(FeederConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(FeederConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;

    topVelocitySignal = topMotor.getVelocity(false);
    bottomVelocitySignal = bottomMotor.getVelocity(false);
    topStatorCurrentSignal = topMotor.getStatorCurrent(false);
    bottomStatorCurrentSignal = bottomMotor.getStatorCurrent(false);
    allSignals =
        List.of(
            topVelocitySignal,
            bottomVelocitySignal,
            topStatorCurrentSignal,
            bottomStatorCurrentSignal);
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOT);
  }

  public void intakeRequest() {
    setStateFromRequest(FeederState.INTAKING);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  public void ejectRequest() {
    setStateFromRequest(FeederState.EJECT);
  }

  public void ballFillingRequest() {
    setStateFromRequest(FeederState.BALL_FILLING);
  }

  @Override
  protected void afterTransition(FeederState newState) {
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
    BaseStatusSignal.refreshAll(allSignals);
    DogLog.log("Feeder/Top/VelocityRPM", topVelocitySignal.getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Bottom/VelocityRPM", bottomVelocitySignal.getValueAsDouble() * 60.0);
    DogLog.log("Feeder/Voltage", getState().getVoltage());

    averageCurrent =
        MathHelpers.average(
            topStatorCurrentSignal.getValueAsDouble(),
            bottomStatorCurrentSignal.getValueAsDouble());
  }

  public double getAverageCurrent() {
    return averageCurrent;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topMotor
        .getConfigurator()
        .apply(
            FeederConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            FeederConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }
}
