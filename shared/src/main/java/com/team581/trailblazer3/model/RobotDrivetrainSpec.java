package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.List;

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
