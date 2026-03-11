package frc.robot.climber;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.simkit.SimKit;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;

public class Climber extends GenericClimber {

  private final TalonFX motor;
  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);
  private double motorPosition;

  public Climber(TalonFX motor) {
    this.motor = motor;

    motor.getConfigurator().apply(ClimberConfig.MOTOR_CONFIG);

    motor.setPosition(0);

    TunablePid.register("Climber", this.motor, ClimberConfig.MOTOR_CONFIG);
  }

  @Override
  public boolean atGoal() {
    return MathUtil.isNear(getState().height, motorPosition, ClimberConfig.TOLERANCE);
  }

  @Override
  public double getHeight() {
    return motorPosition;
  }

  @Override
  public void l1HangingRequest() {
    setStateFromRequest(ClimberState.L1_HANG);
  }

  @Override
  public void l1LineupRequest() {
    setStateFromRequest(ClimberState.L1_LINEUP);
  }

  @Override
  public void l2HangingRequest() {
    setStateFromRequest(ClimberState.L2_HANG);
  }

  @Override
  public void l2LineupRequest() {
    setStateFromRequest(ClimberState.L2_LINEUP);
  }

  @Override
  public void l3HangingRequest() {
    setStateFromRequest(ClimberState.L3_HANG);
  }

  @Override
  public void l3LineupRequest() {
    setStateFromRequest(ClimberState.L3_LINEUP);
  }

  @Override
  public void simulationPeriodic() {
    var climberSimulation =
        SimKit.positionMechanism(
            "Climber",
            mechanism ->
                mechanism
                    .addMotor(motor, ChassisReference.CounterClockwise_Positive)
                    .withMinPosition(ClimberConfig.MIN_HEIGHT)
                    .withMaxPosition(ClimberConfig.MAX_HEIGHT));

    climberSimulation.update();
  }

  @Override
  public void stowRequest() {
    setStateFromRequest(ClimberState.STOWED);
  }

  @Override
  protected void afterTransition(ClimberState newState) {
    motor.setControl(positionRequest.withPosition(newState.height));
  }

  @Override
  protected void collectInputs() {
    motorPosition = motor.getPosition().getValueAsDouble();
    DogLog.log("Climber/TargetHeight", getState().height);
    DogLog.log("Climber/Position", motorPosition);
    DogLog.log("Climber/AtGoal", atGoal());
  }
}
