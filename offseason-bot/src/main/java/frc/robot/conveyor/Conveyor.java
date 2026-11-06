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
    DogLog.log("Floor/SupplyLimitAmps", floorLimits.SupplyCurrentLimit);
    DogLog.log("Floor/StatorLimitAmps", floorLimits.StatorCurrentLimit);
    var tunnelTopLimits =
        ConveyorConfig.TUNNEL_TOP_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
            supplyCurrentLimit);
    tunnelTopMotor.getConfigurator().apply(tunnelTopLimits);
    DogLog.log("Tunnel/Top/SupplyLimitAmps", tunnelTopLimits.SupplyCurrentLimit);
    DogLog.log("Tunnel/Top/StatorLimitAmps", tunnelTopLimits.StatorCurrentLimit);
    var tunnelBottomLimits =
        ConveyorConfig.TUNNEL_BOTTOM_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
            supplyCurrentLimit);
    tunnelBottomMotor.getConfigurator().apply(tunnelBottomLimits);
    DogLog.log("Tunnel/Bottom/SupplyLimitAmps", tunnelBottomLimits.SupplyCurrentLimit);
    DogLog.log("Tunnel/Bottom/StatorLimitAmps", tunnelBottomLimits.StatorCurrentLimit);
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
    DogLog.log("Floor/SupplyVoltage", supplyVoltageSignal.getValueAsDouble());
    DogLog.log("Floor/SupplyCurrent", supplyCurrentSignal.getValueAsDouble());
    DogLog.log("Tunnel/Top/SupplyVoltage", tunnelTopSupplyVoltageSignal.getValueAsDouble());
    DogLog.log("Tunnel/Bottom/SupplyVoltage", tunnelBottomSupplyVoltageSignal.getValueAsDouble());
    DogLog.log("Tunnel/Top/SupplyCurrent", tunnelTopSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Tunnel/Bottom/SupplyCurrent", tunnelBottomSupplyCurrentSignal.getValueAsDouble());

    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double velocityRps = velocitySignal.getValueAsDouble();
    surfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            velocityRps,
            ConveyorConfig.ROLLER_DIAMETER_METERS,
            ConveyorConfig.MOTOR_TO_ROLLER_RATIO);
    double tunnelTopVelocityRps = tunnelTopVelocitySignal.getValueAsDouble();
    double tunnelBottomVelocityRps = tunnelBottomVelocitySignal.getValueAsDouble();

    DogLog.log("Floor/VelocityRPM", velocityRps * 60.0);
    DogLog.log("Floor/SurfaceSpeedMetersPerSecond", surfaceSpeedMetersPerSecond);
    DogLog.log("Tunnel/Top/VelocityRPM", tunnelTopVelocityRps * 60.0);
    DogLog.log("Tunnel/Bottom/VelocityRPM", tunnelBottomVelocityRps * 60.0);
    double metersPerRevolution = ConveyorConfig.TUNNEL_METERS_PER_PULLEY_REVOLUTION.get();
    boolean surfaceSpeedCalibrated =
        Double.isFinite(metersPerRevolution) && metersPerRevolution > 0.0;
    DogLog.log("Tunnel/SurfaceSpeedCalibrated", surfaceSpeedCalibrated);
    DogLog.log(
        "Tunnel/TopSurfaceSpeedMetersPerSecond",
        surfaceSpeedCalibrated
            ? tunnelTopVelocityRps
                / ConveyorConfig.TUNNEL_MOTOR_TO_PULLEY_RATIO
                * metersPerRevolution
            : Double.NaN);
    DogLog.log(
        "Tunnel/BottomSurfaceSpeedMetersPerSecond",
        surfaceSpeedCalibrated
            ? tunnelBottomVelocityRps
                / ConveyorConfig.TUNNEL_MOTOR_TO_PULLEY_RATIO
                * metersPerRevolution
            : Double.NaN);
  }
}
