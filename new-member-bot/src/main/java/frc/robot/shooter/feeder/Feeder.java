package frc.robot.shooter.feeder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final StatusSignal<AngularVelocity> topVelocitySignal;
  private final StatusSignal<AngularVelocity> bottomVelocitySignal;
  private final StatusSignal<Current> topStatorCurrentSignal;
  private final StatusSignal<Current> bottomStatorCurrentSignal;

  public Feeder(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    topMotor.getConfigurator().apply(FeederConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(FeederConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
    // My top
    topVelocitySignal = topMotor.getVelocity(false);
    topStatorCurrentSignal = topMotor.getStatorCurrent(false);
    // My Bottom
    bottomVelocitySignal = bottomMotor.getVelocity(false);
    bottomStatorCurrentSignal = bottomMotor.getStatorCurrent(false);
    // Signal Register
    Signals.forDevice(topMotor).addSignals(topVelocitySignal, topStatorCurrentSignal);
    Signals.forDevice(bottomMotor).addSignals(bottomVelocitySignal, bottomStatorCurrentSignal);
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

  public void ballFillingRequest() {
    setStateFromRequest(FeederState.BALL_FILLING);
  }

  public void ejectRequest() {
    setStateFromRequest(FeederState.EJECT);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FeederState.INTAKING);
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOTING);
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
}
