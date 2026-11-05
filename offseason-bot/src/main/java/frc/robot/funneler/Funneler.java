package frc.robot.funneler;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.signals.Signals;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RollerUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Funneler extends StateMachineSubsystem<FunnelerState> implements PowerManaged {
  private final TalonFX motor;
  private final StatusSignal<Voltage> supplyVoltageSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private double surfaceSpeedMetersPerSecond = 0.0;

  public Funneler(TalonFX motor) {
    super(SubsystemPriority.FUNNELER, FunnelerState.IDLE);
    motor.getConfigurator().apply(FunnelerConfig.MOTOR_CONFIG);
    this.motor = motor;
    velocitySignal = motor.getRotorVelocity(false);
    Signals.forDevice(motor).addSignals(velocitySignal);
    supplyVoltageSignal = motor.getSupplyVoltage(false);
    Signals.forDevice(motor).addSignals(supplyVoltageSignal);
    supplyCurrentSignal = motor.getSupplyCurrent(false);
    Signals.forDevice(motor).addSignals(supplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    var motorLimits =
        FunnelerConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit);
    motor.getConfigurator().apply(motorLimits);
    DogLog.log("Funneler/SupplyLimitAmps", motorLimits.SupplyCurrentLimit);
    DogLog.log("Funneler/StatorLimitAmps", motorLimits.StatorCurrentLimit);
  }

  public void ballFillingRequest() {
    setStateFromRequest(FunnelerState.BALL_FILLING);
  }

  public void feedingRequest() {
    setStateFromRequest(FunnelerState.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(FunnelerState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FunnelerState.INTAKE);
  }

  public void scoreRequest() {
    setStateFromRequest(FunnelerState.SCORE);
  }

  @Override
  protected void afterTransition(FunnelerState newState) {
    switch (newState) {
      case IDLE -> motor.setControl(neutralRequest);
      default -> motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Funneler/SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
    DogLog.log("Funneler/SupplyCurrent", supplyCurrentSignal.getValueAsDouble());

    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double velocityRps = velocitySignal.getValueAsDouble();
    surfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            velocityRps,
            FunnelerConfig.ROLLER_DIAMETER_METERS,
            FunnelerConfig.MOTOR_TO_ROLLER_RATIO);

    DogLog.log("Funneler/VelocityRPM", velocityRps * 60.0);
    DogLog.log("Funneler/SurfaceSpeedMetersPerSecond", surfaceSpeedMetersPerSecond);
  }
}
