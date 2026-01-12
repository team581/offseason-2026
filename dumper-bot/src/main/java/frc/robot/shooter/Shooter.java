package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;

import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter extends StateMachineSubsystem<ShooterState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private double distance = 0;
  private double shootingRpm = 0;
  private double feedingRpm = 0;
  private double leftMotorRpm = 0;
  private double rightMotorRpm = 0;
  public Shooter(TalonFX leftMotor, TalonFX rightMotor) {

    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);
     leftMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
     rightMotor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  public void scoreRequest(double distance) {
    this.distance = distance;
    setStateFromRequest(ShooterState.SCORE);
  }

  public void feedRequest(double distance) {
    this.distance = distance;
    setStateFromRequest(ShooterState.FEEDING);
  }

  public void idleRequest() {
    setStateFromRequest(ShooterState.IDLE);
  }

  @Override
  protected void whileInState(ShooterState state) {
    if (state == ShooterState.SCORE ) {
      velocityRequest.withVelocity(shootingRpm);
    }
    if (state == ShooterState.FEEDING){
      velocityRequest.withVelocity(feedingRpm);
    }
    leftMotor.setControl(velocityRequest);
    rightMotor.setControl(velocityRequest);
  }
    private double shootingDistancetoRPm() {
      return 1.0;
    }
    private double feedingDistancetoRpm() {
      return 1.0;
    }
    @Override
    protected void collectInputs() {
      shootingRpm = Math.min(4000, shootingDistancetoRPm());
      feedingRpm = Math.min(4000, feedingDistancetoRpm());

      leftMotorRpm = leftMotor.getVelocity().getValueAsDouble();
      rightMotorRpm = rightMotor.getVelocity().getValueAsDouble();
    }
    public boolean atGoal() {
      return switch(getState()) {
       case IDLE -> true;
        case SCORE -> {
          yield Math.abs(shootingRpm - leftMotorRpm) <= 50 && Math.abs(shootingRpm - rightMotorRpm) <= 50;
        }
        case FEEDING -> {
          yield Math.abs(feedingRpm - leftMotorRpm) <= 100 && Math.abs(feedingRpm - rightMotorRpm) <= 50;
        }
      };

    }

}
