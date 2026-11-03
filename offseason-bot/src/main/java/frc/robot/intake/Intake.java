package frc.robot.intake;

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
import frc.robot.util.RollerUtil;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> implements PowerManaged {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> leftVelocitySignal;
  private final StatusSignal<AngularVelocity> rightVelocitySignal;

  private final StatusSignal<Current> leftSupplyCurrentSignal;
  private final StatusSignal<Current> rightSupplyCurrentSignal;

  private double leftSurfaceSpeedMetersPerSecond = 0.0;
  private double rightSurfaceSpeedMetersPerSecond = 0.0;

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    leftMotor.getConfigurator().apply(IntakeConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(IntakeConfig.RIGHT_MOTOR_CONFIG);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftVelocitySignal = leftMotor.getRotorVelocity(false);
    leftSupplyCurrentSignal = leftMotor.getSupplyCurrent(false);
    rightVelocitySignal = rightMotor.getRotorVelocity(false);
    rightSupplyCurrentSignal = rightMotor.getSupplyCurrent(false);
    Signals.forDevice(leftMotor).addSignals(leftVelocitySignal, leftSupplyCurrentSignal);
    Signals.forDevice(rightMotor).addSignals(rightVelocitySignal, rightSupplyCurrentSignal);
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    leftMotor
        .getConfigurator()
        .apply(
            IntakeConfig.LEFT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
    rightMotor
        .getConfigurator()
        .apply(
            IntakeConfig.RIGHT_MOTOR_CONFIG.CurrentLimits.withSupplyCurrentLimit(
                supplyCurrentLimit));
  }

  public void ejectRequest() {
    setStateFromRequest(IntakeState.EJECT);
  }

  public void feedRequest() {
    setStateFromRequest(IntakeState.FEED);
  }

  public boolean hasBeenIntaking() {
    if (getState() == IntakeState.INTAKE && timeout(3)) {
      return true;
    }
    return false;
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKE);
  }

  public void scoreRequest() {
    setStateFromRequest(IntakeState.SCORE);
  }

  public void stopEjectingRequest() {
    switch (getState()) {
      case EJECT -> setStateFromRequest(IntakeState.IDLE);
      default -> {}
    }
  }

  public void stopFeedingRequest() {
    switch (getState()) {
      case FEED -> setStateFromRequest(IntakeState.IDLE);
      default -> {}
    }
  }

  public void stopIntakingRequest() {
    switch (getState()) {
      case INTAKE -> setStateFromRequest(IntakeState.IDLE);
      default -> {}
    }
  }

  public void stopScoringRequest() {
    switch (getState()) {
      case SCORE -> setStateFromRequest(IntakeState.IDLE);
      default -> {}
    }
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> {
        leftMotor.setControl(neutralRequest);
        rightMotor.setControl(neutralRequest);
      }
      default -> {
        leftMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
        rightMotor.setControl(voltageRequest.withOutput(newState.getVoltage()));
      }
    }
  }

  @Override
  protected void collectInputs() {
    // Use raw motor rotations/sec so feedback scaling cannot apply gearing twice.
    double leftVelocityRps = leftVelocitySignal.getValueAsDouble();
    leftSurfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            leftVelocityRps,
            IntakeConfig.LEFT_ROLLER_DIAMETER_METERS,
            IntakeConfig.MOTOR_TO_ROLLER_RATIO);
    double rightVelocityRps = rightVelocitySignal.getValueAsDouble();
    rightSurfaceSpeedMetersPerSecond =
        RollerUtil.surfaceSpeedMetersPerSecond(
            rightVelocityRps,
            IntakeConfig.RIGHT_ROLLER_DIAMETER_METERS,
            IntakeConfig.MOTOR_TO_ROLLER_RATIO);

    DogLog.log("Intake/Left/VelocityRPM", leftVelocityRps * 60.0);
    DogLog.log("Intake/Left/SurfaceSpeedMetersPerSecond", leftSurfaceSpeedMetersPerSecond);
    DogLog.log("Intake/Right/VelocityRPM", rightVelocityRps * 60.0);
    DogLog.log("Intake/Right/SurfaceSpeedMetersPerSecond", rightSurfaceSpeedMetersPerSecond);

    DogLog.log("Intake/Left/SupplyCurrent", leftSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Intake/Right/SupplyCurrent", rightSupplyCurrentSignal.getValueAsDouble());
    DogLog.log("Intake/HasBeenIntaking", hasBeenIntaking());
  }
}
