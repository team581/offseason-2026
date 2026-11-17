package frc.robot.config;

import com.ctre.phoenix6.Utils;
import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.config.TrailblazerConfig;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.CompTunerConstants;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public final class CompTrailblazer {
  static final double BUMPER_HALF_SIZE_METERS = Units.inchesToMeters(35.0 / 2);

  public static final EmpiricalDriveCalibration CALIBRATION = TrailblazerConfig.COMP;

  public static Trailblazer create() {
    return create(Utils::getCurrentTimeSeconds);
  }

  public static Trailblazer create(DoubleSupplier time) {
    var frontLeft = CompTunerConstants.FrontLeft;
    var modules =
        ImmutableList.of(
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
    return new Trailblazer(time, robot, CALIBRATION, 0.020, Optional.of(CompAutoField.CLEARANCE));
  }

  private CompTrailblazer() {}
}
