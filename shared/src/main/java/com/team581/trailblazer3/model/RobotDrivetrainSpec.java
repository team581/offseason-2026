package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;

/**
 * Physical inputs for the drivetrain model.
 *
 * @param massKg weigh the complete robot, including battery and bumpers, in kg
 * @param yawMoiKgMetersSq vertical-axis inertia from a CAD mass-properties report, in kg·m²
 * @param wheelRadiusMeters loaded rolling radius; measure travel / wheel radians, in meters
 * @param driveGearRatio motor revolutions per wheel revolution; multiply the drive gear stages
 * @param steerGearRatio motor revolutions per azimuth revolution; multiply the steer gear stages
 * @param modulePositionsMeters module centers relative to robot center, from CAD, in meters
 * @param driveMotor WPILib model for the drive motor and control mode; supplies Kv, Kt, resistance,
 *     free current, and nominal voltage
 * @param steerMotor WPILib model for the steer motor and control mode
 * @param slipCurrentAmps per-drive-motor current where the wheel starts slipping; measure on carpet
 * @param staticFrictionVolts voltage needed to start wheel motion; measure with a slow voltage ramp
 */
public record RobotDrivetrainSpec(
    double massKg,
    double yawMoiKgMetersSq,
    double wheelRadiusMeters,
    double driveGearRatio,
    double steerGearRatio,
    List<Translation2d> modulePositionsMeters,
    DCMotor driveMotor,
    DCMotor steerMotor,
    double slipCurrentAmps,
    double staticFrictionVolts) {
  public RobotDrivetrainSpec {
    modulePositionsMeters = List.copyOf(modulePositionsMeters);
    for (double value :
        new double[] {
          massKg,
          yawMoiKgMetersSq,
          wheelRadiusMeters,
          driveGearRatio,
          steerGearRatio,
          slipCurrentAmps,
          driveMotor.KvRadPerSecPerVolt,
          steerMotor.KvRadPerSecPerVolt
        }) {
      checkArgument(Double.isFinite(value) && value > 0, "Nonpositive drivetrain parameter");
    }
    checkArgument(
        massKg >= 20
            && massKg <= 80
            && wheelRadiusMeters >= 0.035
            && wheelRadiusMeters <= 0.080
            && Double.isFinite(staticFrictionVolts)
            && staticFrictionVolts >= 0
            && staticFrictionVolts < driveMotor.nominalVoltageVolts
            && modulePositionsMeters.size() == 4
            && modulePositionsMeters.stream()
                .noneMatch(p ->
                    !Double.isFinite(p.getNorm())
                        || Math.abs(p.getX()) < 0.18
                        || Math.abs(p.getY()) < 0.18),
        "Drivetrain parameters outside validation bounds");
    double radius =
        modulePositionsMeters.stream().mapToDouble(Translation2d::getNorm).max().orElseThrow();
    checkArgument(radius >= 0.28 && radius <= 0.60, "Module radius outside validation bounds");
  }
}
