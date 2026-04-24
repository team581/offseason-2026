package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.function.Supplier;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  private static final double IS_FLAT_THRESHOLD = 5.0;

  private Supplier<SwerveDriveState> driveStateSupplier;
  private final Debouncer isFlatDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<Angle> rollSignal;
  protected final StatusSignal<LinearAcceleration> accelerationXSignal;
  protected final StatusSignal<LinearAcceleration> accelerationYSignal;
  private final List<BaseStatusSignal> pigeonSignals;

  private boolean isFlatDebounced = false;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;
  private double rawPitch = 0;
  private double rawRoll = 0;
  private double pitchOffset = 0;
  private double rollOffset = 0;

  public BaseImuSubsystem(
      SubsystemPriorityBase priority,
      Pigeon2 pigeon,
      Supplier<SwerveDriveState> driveStateSupplier) {
    super(priority, ImuState.DEFAULT_STATE);

    this.driveStateSupplier = driveStateSupplier;

    pitchSignal = pigeon.getPitch(false);
    rollSignal = pigeon.getRoll(false);
    accelerationXSignal = pigeon.getAccelerationX(false);
    accelerationYSignal = pigeon.getAccelerationY(false);
    pigeonSignals = List.of(pitchSignal, rollSignal, accelerationXSignal, accelerationYSignal);
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

  /**
   * Replace the {@link SwerveDriveState} supplier after construction. Workaround for circular
   * import between Swerve and Imu.
   */
  public void setDriveStateSupplier(Supplier<SwerveDriveState> supplier) {
    this.driveStateSupplier = supplier;
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
    BaseStatusSignal.refreshAll(pigeonSignals);

    driveState = driveStateSupplier.get();
    robotHeading = MathHelpers.angleModulus(driveState.Pose.getRotation().getDegrees());
    robotAngularVelocity = Math.toDegrees(driveState.Speeds.omegaRadiansPerSecond);

    if (RobotBase.isReal()) {
      rawPitch = pitchSignal.getValueAsDouble();
      rawRoll = rollSignal.getValueAsDouble();
    }

    isFlatDebounced =
        isFlatDebouncer.calculate(
            MathUtil.isNear(getPitch(), 0, IS_FLAT_THRESHOLD, -90, 90)
                && MathUtil.isNear(getRoll(), 0, IS_FLAT_THRESHOLD, -180, 180));
  }
}
