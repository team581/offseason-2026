package frc.robot.config;

import com.ctre.phoenix6.Utils;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.CompTunerConstants;
import java.util.List;

public final class CompTrailblazer {
  static final double BUMPER_HALF_SIZE_METERS =
      edu.wpi.first.math.util.Units.inchesToMeters(35.0 / 2);
  // Initial conservative estimates for the velocity-only testbed. Confirm on carpet.
  private static final EmpiricalDriveCalibration CALIBRATION =
      new EmpiricalDriveCalibration(4, 1.6, 0.070);

  public static Trailblazer create() {
    return create(Utils::getCurrentTimeSeconds);
  }

  public static Trailblazer create(com.team581.trailblazer3.runtime.TimeSource time) {
    var frontLeft = CompTunerConstants.FrontLeft;
    var modules =
        List.of(
            frontLeft,
            CompTunerConstants.FrontRight,
            CompTunerConstants.BackLeft,
            CompTunerConstants.BackRight);
    var robot =
        new RobotDrivetrainSpec(
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
    return new Trailblazer(time, robot, CALIBRATION, 0.020);
  }

  private CompTrailblazer() {}
}
