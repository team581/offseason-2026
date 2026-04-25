package com.team581.mechanisms.imu;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team581.math.MathHelpers;
import com.team581.signals.Signals;
import com.team581.util.scheduling.SubsystemPriorityBase;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;

public class BaseImuSubsystem extends StateMachineSubsystem<ImuState> {
  private static final double IS_FLAT_THRESHOLD = 5.0;
  private static final int OFFSET_FILTER_TAPS = 25;

  private Supplier<SwerveDriveState> driveStateSupplier;
  private final Debouncer isFlatDebouncer = new Debouncer(0.5, DebounceType.kRising);

  private final StatusSignal<Angle> pitchSignal;
  private final StatusSignal<Angle> rollSignal;
  protected final StatusSignal<LinearAcceleration> accelerationXSignal;
  protected final StatusSignal<LinearAcceleration> accelerationYSignal;

  private boolean isFlatDebounced = false;

  protected SwerveDriveState driveState = new SwerveDriveState();
  protected double robotHeading = 0;
  protected double robotAngularVelocity = 0;
  private double rawPitch = 0;
  private double rawRoll = 0;
  private final LinearFilter pitchOffsetFilter = LinearFilter.movingAverage(OFFSET_FILTER_TAPS);
  private final LinearFilter rollOffsetFilter = LinearFilter.movingAverage(OFFSET_FILTER_TAPS);
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
    Signals.forDevice(pigeon)
        .addSignals(pitchSignal, rollSignal, accelerationXSignal, accelerationYSignal);
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
    DogLog.log("Imu/Main/RobotHeading", robotHeading, Degrees);
    DogLog.log("Imu/Main/AngularVelocity", robotAngularVelocity, DegreesPerSecond);
    DogLog.log("Imu/Main/Pitch/Raw", rawPitch, Degrees);
    DogLog.log("Imu/Main/Pitch/Offset", pitchOffset, Degrees);
    DogLog.log("Imu/Main/Pitch", getPitch(), Degrees);
    DogLog.log("Imu/Main/Roll", getRoll(), Degrees);
    DogLog.log("Imu/Main/Roll/Raw", rawRoll, Degrees);
    DogLog.log("Imu/Main/Roll/Offset", rollOffset, Degrees);
    DogLog.log("Imu/Main/IsFlatDebounced", isFlatDebounced);
  }

  @Override
  protected void collectInputs() {
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

    // Only feed the offset filter when the robot is flat. During (and immediately after) a bump
    // crossing the IMU is noisy, so we freeze the offset until things settle, then slowly absorb
    // the new readings as the moving average refills.
    if (isFlatDebounced) {
      pitchOffset = pitchOffsetFilter.calculate(rawPitch);
      rollOffset = rollOffsetFilter.calculate(rawRoll);
    }
  }
}
