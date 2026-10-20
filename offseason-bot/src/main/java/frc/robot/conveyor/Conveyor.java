package frc.robot.conveyor;

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
import frc.robot.util.scheduling.SubsystemPriority;

public class Conveyor extends StateMachineSubsystem<ConveyorState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> topVelocitySignal;
  private final StatusSignal<AngularVelocity> bottomVelocitySignal;

  private final StatusSignal<Current> topSupplyCurrentSignal;
  private final StatusSignal<Current> bottomSupplyCurrentSignal;

  private double topSurfaceSpeedMetersPerSecond = 0.0;
  private double bottomSurfaceSpeedMetersPerSecond = 0.0;

  public Conveyor(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.CONVEYOR, ConveyorState.IDLE);
    topMotor.getConfigurator().apply(ConveyorConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(ConveyorConfig.BOTTOM_MOTOR_CONFIG);

    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;

    topVelocitySignal = topMotor.getVelocity(false);
    topSupplyCurrentSignal = topMotor.getSupplyCurrent(false);
    bottomVelocitySignal = bottomMotor.getVelocity(false);
    bottomSupplyCurrentSignal = bottomMotor.getSupplyCurrent(false);
    Signals.forDevice(topMotor).addSignals(topVelocitySignal, topSupplyCurrentSignal);
    Signals.forDevice(bottomMotor).addSignals(bottomVelocitySignal, bottomSupplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    topMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    bottomMotor
        .getConfigurator()
        .apply(
            ConveyorConfig.BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public void ballFillingRequest() {
    setStateFromRequest(ConveyorState.BALL_FILLING);
  }

  public void ejectRequest() {
    setStateFromRequest(ConveyorState.EJECT);
  }

  public void feedRequest() {
    setStateFromRequest(ConveyorState.FEED);
  }

  public void idleRequest() {
    setStateFromRequest(ConveyorState.IDLE);
  }

  public void initialShotRequest() {
    setStateFromRequest(ConveyorState.INITIAL_SHOT);
  }

  public void intakeRequest() {
    setStateFromRequest(ConveyorState.INTAKE);
  }

  public void scoreRequest() {
    setStateFromRequest(ConveyorState.SCORE);
  }

  @Override
  protected void afterTransition(ConveyorState newState) {
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
    // Phoenix velocity is roller rotations/sec after SensorToMechanismRatio.
    double topVelocityRps = topVelocitySignal.getValueAsDouble();
    topSurfaceSpeedMetersPerSecond =
        topVelocityRps * Math.PI * ConveyorConfig.TOP_ROLLER_DIAMETER_METERS;
    double bottomVelocityRps = bottomVelocitySignal.getValueAsDouble();
    bottomSurfaceSpeedMetersPerSecond =
        bottomVelocityRps * Math.PI * ConveyorConfig.BOTTOM_ROLLER_DIAMETER_METERS;

    DogLog.log("Conveyor/Top/VelocityRPM", topVelocityRps * 60.0);
    DogLog.log("Conveyor/Top/SurfaceSpeedMetersPerSecond", topSurfaceSpeedMetersPerSecond);
    DogLog.log("Conveyor/Bottom/VelocityRPM", bottomVelocityRps * 60.0);
    DogLog.log("Conveyor/Bottom/SurfaceSpeedMetersPerSecond", bottomSurfaceSpeedMetersPerSecond);
  }
}
