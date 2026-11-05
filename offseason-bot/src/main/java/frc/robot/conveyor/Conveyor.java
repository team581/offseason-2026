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
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.RollerUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Conveyor extends StateMachineSubsystem<ConveyorState> implements PowerManaged {

  private final TalonFX motor;
  private final TalonFX tunnelTopMotor;
  private final TalonFX tunnelBottomMotor;

  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<Voltage> supplyVoltageSignal;
  private final StatusSignal<Voltage> tunnelTopSupplyVoltageSignal;
  private final StatusSignal<Voltage> tunnelBottomSupplyVoltageSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularVelocity> tunnelTopVelocitySignal;
  private final StatusSignal<AngularVelocity> tunnelBottomVelocitySignal;

  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> tunnelTopSupplyCurrentSignal;
  private final StatusSignal<Current> tunnelBottomSupplyCurrentSignal;

  private double surfaceSpeedMetersPerSecond = 0.0;

  public Conveyor(TalonFX motor, TalonFX tunnelTopMotor, TalonFX tunnelBottomMotor) {
    super(SubsystemPriority.CONVEYOR, ConveyorState.IDLE);
    motor.getConfigurator().apply(ConveyorConfig.MOTOR_CONFIG);
    tunnelTopMotor.getConfigurator().apply(ConveyorConfig.TUNNEL_TOP_MOTOR_CONFIG);
    tunnelBottomMotor.getConfigurator().apply(ConveyorConfig.TUNNEL_BOTTOM_MOTOR_CONFIG);

    this.motor = motor;
    this.tunnelTopMotor = tunnelTopMotor;
    this.tunnelBottomMotor = tunnelBottomMotor;

    velocitySignal = motor.getRotorVelocity(false);
    supplyCurrentSignal = motor.getSupplyCurrent(false);
    tunnelTopVelocitySignal = tunnelTopMotor.getRotorVelocity(false);
    tunnelBottomVelocitySignal = tunnelBottomMotor.getRotorVelocity(false);
    tunnelTopSupplyCurrentSignal = tunnelTopMotor.getSupplyCurrent(false);
    tunnelBottomSupplyCurrentSignal = tunnelBottomMotor.getSupplyCurrent(false);
    Signals.forDevice(motor).addSignals(velocitySignal, supplyCurrentSignal);
    Signals.forDevice(tunnelTopMotor)
        .addSignals(tunnelTopVelocitySignal, tunnelTopSupplyCurrentSignal);
    Signals.forDevice(tunnelBottomMotor)
        .addSignals(tunnelBottomVelocitySignal, tunnelBottomSupplyCurrentSignal);
    supplyVoltageSignal = motor.getSupplyVoltage(false);
    Signals.forDevice(motor).addSignals(supplyVoltageSignal);
    tunnelTopSupplyVoltageSignal = tunnelTopMotor.getSupplyVoltage(false);
    tunnelBottomSupplyVoltageSignal = tunnelBottomMotor.getSupplyVoltage(false);
    Signals.forDevice(tunnelTopMotor).addSignals(tunnelTopSupplyVoltageSignal);
    Signals.forDevice(tunnelBottomMotor).addSignals(tunnelBottomSupplyVoltageSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    var floorLimits =
        ConveyorConfig.MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(supplyCurrentLimit);
    motor.getConfigurator().apply(floorLimits);
    DogLog.log("Conveyor/Floor/SupplyLimitAmps", floorLimits.SupplyCurrentLimit);
    DogLog.log("Conveyor/Floor/StatorLimitAmps", floorLimits.StatorCurrentLimit);
    var tunnelTopLimits =
        ConveyorConfig.TUNNEL_TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
            supplyCurrentLimit);
    tunnelTopMotor.getConfigurator().apply(tunnelTopLimits);
    DogLog.log("Conveyor/TunnelTop/SupplyLimitAmps", tunnelTopLimits.SupplyCurrentLimit);
    DogLog.log("Conveyor/TunnelTop/StatorLimitAmps", tunnelTopLimits.StatorCurrentLimit);
    var tunnelBottomLimits =
        ConveyorConfig.TUNNEL_BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
            supplyCurrentLimit);
    tunnelBottomMotor.getConfigurator().apply(tunnelBottomLimits);
    DogLog.log("Conveyor/TunnelBottom/SupplyLimitAmps", tunnelBottomLimits.SupplyCurrentLimit);
    DogLog.log("Conveyor/TunnelBottom/StatorLimitAmps", tunnelBottomLimits.StatorCurrentLimit);
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
        motor.setControl(neutralRequest);
        tunnelTopMotor.setControl(neutralRequest);
        tunnelBottomMotor.setControl(neutralRequest);
      }
      default -> {
        motor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        tunnelTopMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        tunnelBottomMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    DogLog.log("Conveyor/Floor/SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
    DogLog.log("Conveyor/Floor/SupplyCurrent", supplyCurrentSignal.getValueAsDouble());
    DogLog.log("Conveyor/TunnelTop/SupplyVoltage", tunnelTopSupplyVoltageSignal.getValueAsDouble());
    DogLog.log(
        "Conveyor/TunnelBottom/SupplyVoltage", tunnelBottomSupplyVoltageSignal.getValueAsDouble());
    DogLog.log("Conveyor/TunnelTop/SupplyCurrent", tunnelTopSupplyCurrentSignal.getValueAsDouble());
    DogLog.log(
        "Conveyor/TunnelBottom/SupplyCurrent", tunnelBottomSupplyCurrentSignal.getValueAsDouble());

    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double velocityRps = velocitySignal.getValueAsDouble();
    surfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            velocityRps,
            ConveyorConfig.ROLLER_DIAMETER_METERS,
            ConveyorConfig.MOTOR_TO_ROLLER_RATIO);
    double tunnelTopVelocityRps = tunnelTopVelocitySignal.getValueAsDouble();
    double tunnelBottomVelocityRps = tunnelBottomVelocitySignal.getValueAsDouble();

    DogLog.log("Conveyor/Floor/VelocityRPM", velocityRps * 60.0);
    DogLog.log("Conveyor/Floor/SurfaceSpeedMetersPerSecond", surfaceSpeedMetersPerSecond);
    DogLog.log("Conveyor/TunnelTop/VelocityRPM", tunnelTopVelocityRps * 60.0);
    DogLog.log("Conveyor/TunnelBottom/VelocityRPM", tunnelBottomVelocityRps * 60.0);
  }
}
