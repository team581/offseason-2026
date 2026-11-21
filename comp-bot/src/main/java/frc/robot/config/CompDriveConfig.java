package frc.robot.config;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.config.TrailblazerConfig;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.CompTunerConstants;

public final class CompDriveConfig {
  static final double BUMPER_HALF_SIZE_METERS = Units.inchesToMeters(35.0 / 2);

  public static final double FOLLOWER_PERIOD_SECONDS = 0.020;

  public static final EmpiricalDriveCalibration CALIBRATION = TrailblazerConfig.COMP;

  public static final RobotDrivetrainSpec DRIVE_SPEC = driveSpec();

  private static RobotDrivetrainSpec driveSpec() {
    var frontLeft = CompTunerConstants.FrontLeft;
    var modules =
        ImmutableList.of(
            frontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight);
    return new RobotDrivetrainSpec(
        61.2,
        5.20,
        frontLeft.WheelRadius,
        frontLeft.DriveMotorGearRatio,
        frontLeft.SteerMotorGearRatio,
        modules.stream()
            .map(module -> new Translation2d(module.LocationX, module.LocationY))
            .toList(),
        DCMotor.getKrakenX60Foc(1),
        DCMotor.getKrakenX44Foc(1),
        frontLeft.SlipCurrent,
        0.20);
  }

  private CompDriveConfig() {}
}
