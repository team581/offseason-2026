package frc.robot.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachineSubsystem;

import frc.robot.config.FeatureFlags;
import frc.robot.util.scheduling.SubsystemPriority;

public class Shooter2 extends StateMachineSubsystem<ShooterState> {
private final TalonFX leftMotor;
private final TalonFX rightMotor;

 private static double distanceToScoringRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.SCORING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_SCORE_RPM.get(distance);
  }

  private static double distanceToFeedingRpm(double distance) {
    return FeatureFlags.REGRESSION_MODEL.getAsBoolean()
        ? ShooterConfig.FEEDING_REGRESSION_MODEL.calculate(distance)
        : ShooterConfig.DISTANCE_TO_FEEDING_RPM.get(distance);
      }

private final VelocityTorqueCurrentFOC voltageRequest =
      new VelocityTorqueCurrentFOC(0).withLimitReverseMotion(true);

private double scoreDistance = 0;
private double feedDistance = 0;

private double shootingRpm = 0;
private double feedingRpm = 0;
private double leftMotorRpm = 0;
private double rightMotorRpm = 0;

public Shooter2(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SHOOTER, ShooterState.IDLE);

 leftMotor.getConfigurator().apply(ShooterConfig.LEFT_MOTOR_CONFIGS);
    rightMotor.getConfigurator().apply(ShooterConfig.RIGHT_MOTOR_CONFIG);

    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
}
public void scoreRequest(double distance) {
    this.scoreDistance = distance;

    setStateFromRequest(ShooterState.SCORE);
}



}
