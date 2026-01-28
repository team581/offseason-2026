package frc.robot.shooter_hood;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import com.team581.util.tuning.TunablePid;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;

public class ShooterHood extends StateMachineSubsystem<ShooterHoodState> {
  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest =
  new PositionVoltage(0).withEnableFOC(false);

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE =
  TunableInterpolatingDoubleTreeMap.ofEntries(
    "ShooterHood/DistanceToScore", Map.entry(0.0, 0.0));
    private static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED =
    TunableInterpolatingDoubleTreeMap.ofEntries(
      "ShooterHood/DistanceToFeed", Map.entry(0.0, 0.0));
      private static final double MAX_ANGLE = 100;
      private static final double MIN_ANGLE = 0;
      private static final double IDLE_ANGLE = 0.0;
  private double hubDistance = 0;
  private double feedDistance = 0;
  private double measuredAngle = 0;
  private double hubAngle = 0;
  private double feedAngle = 0;
  private double statorCurrent = 0;

  public ShooterHood(TalonFX motor) {
    super(SubsystemPriority.SHOOTER_HOOD, ShooterHoodState.UNHOMED);

    var config =
        new TalonFXConfiguration()
            // TODO: Figure out gearing ratio
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKS(0));
    motor.getConfigurator().apply(config);

    this.motor = motor;

    TunablePid.register("ShooterHood", motor, config);
  }

  public void scoreRequest(double distance) {
    this.hubDistance = distance;
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.SCORING);
    }
  }

  public void feedRequest(double distance) {
    this.feedDistance = distance;
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.FEEDING);
    }
  }

  public void idleRequest() {
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(ShooterHoodState.IDLE);
    }
  }

  public void homingRequest() {
    setStateFromRequest(ShooterHoodState.HOMING);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNHOMED, HOMING -> false;
      case IDLE -> MathUtil.isNear(0, measuredAngle, 1);
      case FEEDING -> MathUtil.isNear(feedAngle, measuredAngle, 1);
      case SCORING -> MathUtil.isNear(hubAngle, measuredAngle, 1);
    };
  }

  @Override
  protected void collectInputs() {
    measuredAngle = motor.getPosition().getValueAsDouble();
    statorCurrent = motor.getStatorCurrent().getValueAsDouble();
    hubAngle = DISTANCE_TO_SCORE.get(hubDistance);
    feedAngle = DISTANCE_TO_FEED.get(feedDistance);

    DogLog.log("ShooterHood/MeasuredAngle", measuredAngle);
    DogLog.log("ShooterHood/FeedingAngle", feedAngle);
    DogLog.log("ShooterHood/StatorCurrent", statorCurrent);
    switch (getState()) {
      case UNHOMED, HOMING -> {
        statorCurrent = motor.getStatorCurrent().getValueAsDouble();
      }
      case SCORING -> measuredAngle = hubAngle;
      case FEEDING -> measuredAngle = feedAngle;
      case IDLE -> {}
    }
  }

  @Override
  protected ShooterHoodState getNextState(ShooterHoodState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (statorCurrent >= 20.0) {
          motor.setPosition(0);
          yield ShooterHoodState.IDLE;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(ShooterHoodState newState) {
    switch (newState) {
      case HOMING -> motor.setVoltage(0);

      case UNHOMED -> motor.disable();

      case IDLE -> {
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(IDLE_ANGLE)));
        DogLog.log("ShooterHood/CurrentSetpoint", 0);
      }

      default -> {}
    }
  }

  @Override
  protected void whileInState(ShooterHoodState state) {
    switch (state) {
      case SCORING -> {
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(hubAngle)));
        DogLog.log("ShooterHood/CurrentSetpoint", hubAngle);
      }

      case FEEDING -> {
        motor.setControl(positionVoltageRequest.withPosition(Units.degreesToRotations(feedAngle)));
        DogLog.log("ShooterHood/CurrentSetpoint", feedAngle);
      }

      default -> {}
    }
  }

  @Override
  public void simulationPeriodic() {
    var shooterHoodSimulation =
        SimKit.positionMechanism(
            "ShooterHood",
            (mechanism) ->
                mechanism
                    .addMotor(motor)
                    .withMaxPosition(Units.degreesToRotations(MAX_ANGLE))
                    .withMinPosition(Units.degreesToRotations(MIN_ANGLE)));

    if (getState() == ShooterHoodState.UNHOMED || getState() == ShooterHoodState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ShooterHoodState.IDLE);
    }

    shooterHoodSimulation.update();
  }
}
