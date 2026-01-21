package frc.robot.imu;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.team581.mechanisms.imu.BaseImuSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import frc.robot.util.scheduling.SubsystemPriority;

public class Imu extends BaseImuSubsystem {
  BuiltInAccelerometer builtInAccelerometer = new BuiltInAccelerometer();

  double builtInPrevXAccel = 0.0;

  double builtInPrevYAccel = 0.0;
  double pigeonXPrevAccel = 0.0;
  double pigeonYPrevAccel = 0.0;
  LinearFilter builtInXAccelFilter = LinearFilter.movingAverage(10);

  LinearFilter builtInYAccelFilter = LinearFilter.movingAverage(10);
  LinearFilter pigeonXAccelFilter = LinearFilter.movingAverage(10);

  LinearFilter pigeonYAccelFilter = LinearFilter.movingAverage(10);

  public Imu(SwerveDrivetrain<?, ?, ?> drivetrain) {
    super(SubsystemPriority.IMU, drivetrain);
  }

  @Override
  public void collectInputs() {
    double builtInFilteredXAccel = builtInXAccelFilter.calculate(builtInAccelerometer.getX());
    double builtInFilteredYAccel = builtInYAccelFilter.calculate(builtInAccelerometer.getY());

    double pigeonFilteredXAccel =
        pigeonXAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble());
    double pigeonFilteredYAccel =
        pigeonYAccelFilter.calculate(drivetrain.getPigeon2().getAccelerationY().getValueAsDouble());

    // Calculates the jerk in the X and Y directions
    // Divides by .02 because default loop timing is 20ms
    double builtInXJerk = (builtInFilteredXAccel - builtInPrevXAccel) / 0.02;
    double builtInYJerk = (builtInFilteredYAccel - builtInPrevYAccel) / 0.02;

    double pigeonXJerk = (pigeonFilteredXAccel - pigeonXPrevAccel) / 0.02;
    double pigeonYJerk = (pigeonFilteredYAccel - pigeonYPrevAccel) / 0.02;

    builtInPrevXAccel = builtInFilteredXAccel;
    builtInPrevYAccel = builtInFilteredYAccel;

    pigeonXPrevAccel = pigeonFilteredXAccel;
    pigeonYPrevAccel = pigeonFilteredYAccel;

    DogLog.log("Imu/BuildInAccelerometer/XJerk", builtInXJerk);
    DogLog.log("Imu/BuildInAccelerometer/YJerk", builtInYJerk);
    DogLog.log("Imu/BuildInAccelerometer/HypotJerk", Math.hypot(builtInXJerk, builtInYJerk));

    DogLog.log("Imu/Pigeon/XJerk", pigeonXJerk);
    DogLog.log("Imu/Pigeon/YJerk", pigeonYJerk);
    DogLog.log("Imu/Pigeon/HypotAccel", Math.hypot(pigeonXJerk, pigeonYJerk));
  }
}
