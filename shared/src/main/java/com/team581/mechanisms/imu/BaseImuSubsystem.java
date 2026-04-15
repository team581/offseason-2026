package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.RobotBase;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  private static final double IS_FLAT_THRESHOLD = 5.0;

  protected final SwerveDrivetrain<?, ?, ?> drivetrain;
  private final Debouncer isFlatDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private boolean isFlatDebounced = false;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;
  private double rawPitch = 0;
  private double rawRoll = 0;
  private double pitchOffset = 0;
  private double rollOffset = 0;

  public BaseImuSubsystem(SubsystemPriorityBase priority, SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(priority, ImuState.DEFAULT_STATE);

    this.drivetrain = drivetrain;
  }

  public double getPitch() {
    return rawPitch - pitchOffset;
  }

  public double getRobotAngularVelocity() {
    return robotAngularVelocity;
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public double getRoll() {
    return rawRoll - rollOffset;
  }

  public boolean isFlatDebounced() {
    return isFlatDebounced;
  }

  public void resetPitchAndRoll() {
    pitchOffset = rawPitch;
    rollOffset = rawRoll;
  }

  // USE FOR SIM ONLY!!!
  public void setPitch(double newPitch) {
    if (RobotBase.isSimulation()) {
      rawPitch = newPitch;
    }
  }

  // USE FOR SIM ONLY!!!
  public void setRoll(double newRoll) {
    if (RobotBase.isSimulation()) {
      rawRoll = newRoll;
    }
  }

  @Override
  public void whileInState(ImuState currentState) {
    DogLog.log("Imu/RobotHeading", robotHeading, Degrees);
    DogLog.log("Imu/AngularVelocity", robotAngularVelocity, DegreesPerSecond);
    DogLog.log("Imu/Pitch", getPitch(), Degrees);
    DogLog.log("Imu/Roll", getRoll(), Degrees);
    DogLog.log("Imu/IsFlatDebounced", isFlatDebounced);
  }

  @Override
  protected void collectInputs() {
    driveState = drivetrain.getState();
    robotHeading = MathHelpers.angleModulus(driveState.Pose.getRotation().getDegrees());
    robotAngularVelocity = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);

    if (RobotBase.isReal()) {
      rawPitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
      rawRoll = drivetrain.getPigeon2().getRoll().getValueAsDouble();
    }

    isFlatDebounced =
        isFlatDebouncer.calculate(
            MathUtil.isNear(getPitch(), 0, IS_FLAT_THRESHOLD, -90, 90)
                && MathUtil.isNear(getRoll(), 0, IS_FLAT_THRESHOLD, -180, 180));
  }
}
