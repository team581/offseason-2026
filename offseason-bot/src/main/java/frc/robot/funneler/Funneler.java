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
import frc.robot.util.scheduling.SubsystemPriority;

public class Funneler extends StateMachineSubsystem<FunnelerState> implements PowerManaged {
  private final TalonFX motor;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private double surfaceSpeedMetersPerSecond = 0.0;

  public Funneler(TalonFX motor) {
    super(SubsystemPriority.FUNNELER, FunnelerState.IDLE);
    motor.getConfigurator().apply(FunnelerConfig.MOTOR_CONFIG);
    this.motor = motor;
    velocitySignal = motor.getVelocity(false);
    Signals.forDevice(motor).addSignals(velocitySignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    motor
        .getConfigurator()
        .apply(
            FunnelerConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit));
  }

  public void ballFillingRequest() {
    setStateFromRequest(FunnelerState.BALL_FILLING);
  }

  public void idleRequest() {
    setStateFromRequest(FunnelerState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FunnelerState.INTAKE);
  }

  public void shootingRequest() {
    setStateFromRequest(FunnelerState.SHOOT);
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
    // Phoenix velocity is roller rotations/sec after SensorToMechanismRatio.
    double velocityRps = velocitySignal.getValueAsDouble();
    surfaceSpeedMetersPerSecond = velocityRps * Math.PI * FunnelerConfig.ROLLER_DIAMETER_METERS;

    DogLog.log("Funneler/VelocityRPM", velocityRps * 60.0);
    DogLog.log("Funneler/SurfaceSpeedMetersPerSecond", surfaceSpeedMetersPerSecond);
  }
}
