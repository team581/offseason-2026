package frc.robot.dye_rotor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class DyeRotor extends StateMachineSubsystem<DyeRotorState> {
  private static final int RPM_TOLERANCE_HORIZONTAL = 100;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("DyeRotor/Horizontal/JamCurrentThreshold", 75.0);

  private final TalonFX rotorMotor;
  private final TalonFX horizontalMotor;
  private final TalonFX verticalMotor;

  private final VelocityVoltage rotorVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private final VelocityVoltage horizontalVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(false);

  private double rotorRawCurrent = 0.0;
  private double rotorFilteredCurrent = 0.0;
  private double rotorShootingRpm = 0;
  private double horizontalShootingRpm = 0;
  private double warmupRpm = 0;
  private double horizontalUnjamRpm = 0;
  private double rotorMotorRpm = 0;
  private double horizontalMotorRpm = 0;
  private double rotorUnjamRpm = -0;

  public DyeRotor(TalonFX rotorMotor, TalonFX horizontalMotor, TalonFX verticalMotor) {
    super(SubsystemPriority.DYE_ROTOR, DyeRotorState.IDLE);

    var rotorConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.0)
                    .withMotionMagicAcceleration(0.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKS(0.0).withKA(0.0));
    var verticalConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(2))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive));
    var horizontalConfigs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.0)
                    .withMotionMagicAcceleration(0.0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(100)
                    .withSupplyCurrentLimit(100))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0.0).withKV(0.0).withKS(0.0));

    rotorMotor.getConfigurator().apply(rotorConfigs);
    horizontalMotor.getConfigurator().apply(horizontalConfigs);
    verticalMotor.getConfigurator().apply(verticalConfigs);

    TunablePid.register("DyeRotor/Rotor", rotorMotor, rotorConfigs);
    TunablePid.register("DyeRotor/Horizontal", horizontalMotor, horizontalConfigs);
    TunablePid.register("DyeRotor/Vertical", verticalMotor, verticalConfigs);

    this.rotorMotor = rotorMotor;
    this.horizontalMotor = horizontalMotor;
    this.verticalMotor = verticalMotor;
  }

  public void shootRequest() {
    setStateFromRequest(DyeRotorState.SHOOTING);
  }

  public void warmupRequest() {
    setStateFromRequest(DyeRotorState.WARMUP);
  }

  public void unjamRequest() {
    setStateFromRequest(DyeRotorState.UNJAM);
  }

  public void idleRequest() {
    setStateFromRequest(DyeRotorState.IDLE);
  }

  @Override
  protected void whileInState(DyeRotorState state) {
    DogLog.log("DyeRotor/Rotor/RPM", rotorMotorRpm);
    DogLog.log("DyeRotor/Rotor/GoalShootingRPM", rotorShootingRpm);
    DogLog.log("DyeRotor/Rotor/Voltage", rotorMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Horizontal/RPM", horizontalShootingRpm);
    DogLog.log("DyeRotor/Horizontal/GoalShootingRPM", horizontalShootingRpm);
    DogLog.log("DyeRotor/Horizontal/Voltage", horizontalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/Vertical/Voltage", verticalMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("DyeRotor/AtGoal", atGoal());

    switch (state) {
      case SHOOTING -> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(rotorShootingRpm));
        horizontalMotor.setControl(horizontalVelocityRequest.withVelocity(horizontalShootingRpm));
        verticalMotor.setVoltage(getState().volts);
      }
      case WARMUP -> {
        rotorMotor.disable();
        horizontalMotor.setControl(horizontalVelocityRequest.withVelocity(warmupRpm));
        verticalMotor.disable();
      }
      case UNJAM -> {
        rotorMotor.setControl(rotorVelocityRequest.withVelocity(rotorUnjamRpm));
        horizontalMotor.setControl(horizontalVelocityRequest.withVelocity(horizontalUnjamRpm));
        verticalMotor.disable();
      }
      case IDLE -> {
        rotorMotor.disable();
        horizontalMotor.disable();
        verticalMotor.disable();
      }
    }
  }

  @Override
  protected void collectInputs() {
    rotorRawCurrent = rotorMotor.getStatorCurrent().getValueAsDouble();
    rotorFilteredCurrent = currentFilter.calculate(rotorRawCurrent);

    rotorMotorRpm = rotorMotor.getVelocity().getValueAsDouble() * 60.0;
    horizontalMotorRpm = horizontalMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public boolean atGoal() {
    return switch (getState()) {
      case IDLE -> true;
      case UNJAM -> timeout(1) || !isJammed();
      case SHOOTING -> true;
      case WARMUP -> MathUtil.isNear(horizontalMotorRpm, warmupRpm, RPM_TOLERANCE_HORIZONTAL);
    };
  }

  public boolean isJammed() {
    return rotorFilteredCurrent > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    var rotorSimulation =
        SimKit.velocityMechanism("DyeRotor/Rotor", (mechanism) -> mechanism.addMotor(rotorMotor));
    var horizontalSimulation =
        SimKit.velocityMechanism(
            "DyeRotor/Horizontal", (mechanism) -> mechanism.addMotor(horizontalMotor));
    var verticalSimulation =
        SimKit.velocityMechanism(
            "DyeRotor/Vertical", (mechanism) -> mechanism.addMotor(verticalMotor));

    rotorSimulation.update();
    horizontalSimulation.update();
    verticalSimulation.update();
  }
}
