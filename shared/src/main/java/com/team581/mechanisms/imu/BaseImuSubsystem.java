package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  protected final SwerveDrivetrain<?, ?, ?> drivetrain;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;

  public BaseImuSubsystem(SubsystemPriorityBase priority, SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(priority, ImuState.DEFAULT_STATE);

    this.drivetrain = drivetrain;
  }

  public double getRobotAngularVelocity() {
    return robotAngularVelocity;
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  @Override
  public void whileInState(ImuState currentState) {
    DogLog.log("Imu/RobotHeading", robotHeading, Degrees);
    DogLog.log("Imu/AngularVelocity", robotAngularVelocity, DegreesPerSecond);
  }

  @Override
  protected void collectInputs() {
    driveState = drivetrain.getState();
    robotHeading = MathHelpers.angleModulus(driveState.Pose.getRotation().getDegrees());
    robotAngularVelocity = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);
  }
}
