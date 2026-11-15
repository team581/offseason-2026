package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import edu.wpi.first.math.MathUtil;
import java.util.OptionalDouble;

public record DrivetrainLimits(
    RobotDrivetrainSpec robot,
    double maxSpeed,
    double maxWheelSpeed,
    double freeSpeed,
    double braking,
    double friction,
    double lateral,
    double maxOmega,
    double maxAlpha,
    double turnRate,
    double lag,
    double moduleRadius,
    double inertiaShare,
    double accelerationCeiling) {
  public static DrivetrainLimits derive(
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      MotionLimits caps) {
    return derive(robot, calibration, nominalDt, caps, 1.0);
  }

  public static DrivetrainLimits derive(
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      MotionLimits caps,
      double scale) {
    var modulePositions = robot.modulePositionsMeters();
    if (modulePositions.isEmpty()) {
      throw new IllegalArgumentException("At least one module position is required");
    }
    double radius = 0.0;
    double squaredRadii = 0.0;
    for (int i = 0; i < modulePositions.size(); i++) {
      var position = modulePositions.get(i);
      double normSq = position.dot(position);
      squaredRadii += normSq;
      radius = Math.max(radius, Math.sqrt(normSq));
    }
    double voltage = robot.driveMotor().nominalVoltageVolts;
    double free =
        (voltage - robot.staticFrictionVolts())
            * robot.driveMotor().KvRadPerSecPerVolt
            * robot.wheelRadiusMeters()
            / robot.driveGearRatio();
    double wheelSpeed = 0.91 * free * scale;
    double max = Math.min(wheelSpeed, caps.maxVelocityMetersPerSecond());
    double friction = calibration.frictionLimit() * scale;
    double lag = calibration.driveResponseLagSeconds();
    double raw = motorAccel(robot, free, 0, OptionalDouble.empty(), OptionalDouble.empty()) / 0.90;
    double steerEffectiveVoltage =
        Math.max(
            0,
            robot.steerMotor().nominalVoltageVolts
                - Math.min(
                    robot.staticFrictionVolts(), 0.5 * robot.steerMotor().nominalVoltageVolts));
    return new DrivetrainLimits(
        robot,
        max,
        wheelSpeed,
        free,
        Math.min(
            calibration.maxBrakingDeceleration() * scale, caps.maxBrakingMetersPerSecondSquared()),
        friction,
        0.85 * friction,
        Math.min(0.85 * wheelSpeed / radius, caps.maxAngularVelocityRadiansPerSecond()),
        Math.min(
            0.85 * robot.massKg() * Math.min(raw, friction) * radius / robot.yawMoiKgMetersSq(),
            caps.maxAngularAccelerationRadiansPerSecondSquared()),
        Math.min(
            0.5
                * steerEffectiveVoltage
                * robot.steerMotor().KvRadPerSecPerVolt
                / robot.steerGearRatio(),
            0.50 / (lag + nominalDt / 2)),
        lag,
        radius,
        Math.min(1, 4 * robot.yawMoiKgMetersSq() / (robot.massKg() * squaredRadii)),
        caps.maxAccelerationMetersPerSecondSquared());
  }

  public double acceleration(double speed, OptionalDouble supply, OptionalDouble restingVoltage) {
    return Math.min(
        accelerationCeiling, motorAccel(robot, freeSpeed, speed, supply, restingVoltage));
  }

  private static double motorAccel(
      RobotDrivetrainSpec robot,
      double freeSpeed,
      double speed,
      OptionalDouble supply,
      OptionalDouble restingVoltage) {
    var motor = robot.driveMotor();
    double nominal = motor.nominalVoltageVolts;
    double voltage = restingVoltage.orElse(nominal);
    double emf =
        robot.staticFrictionVolts() + (nominal - robot.staticFrictionVolts()) * speed / freeSpeed;
    double resistance = motor.rOhms;
    double batteryResistance = 0.018;
    double inverterEfficiency = 0.95;
    double current =
        Math.min(
            robot.slipCurrentAmps(),
            Math.max(0, voltage - emf) / (resistance + 4 * batteryResistance / inverterEfficiency));
    if (supply.isPresent()) {
      double ceiling = supply.getAsDouble();
      double bus = Math.max(7, voltage - 4 * ceiling * batteryResistance);
      double power = inverterEfficiency * bus * ceiling;
      current =
          Math.min(current, 2 * power / (emf + Math.sqrt(emf * emf + 4 * resistance * power)));
    }
    return 0.90
        * 4
        * 0.90
        * robot.driveGearRatio()
        * motor.KtNMPerAmp
        * Math.max(0, current - motor.freeCurrentAmps)
        / (robot.massKg() * robot.wheelRadiusMeters());
  }

  public double availableTraction(double omega, double alpha) {
    return Math.max(
        0, friction - moduleRadius * Math.hypot(inertiaShare * alpha, 0.12 * omega * omega));
  }

  public double rotationSpeed(double velocity) {
    double share = MathUtil.clamp(1 - velocity / 0.50, 0, 1);
    return Math.min(
        maxOmega,
        Math.min(
            (0.45 + 0.55 * share) * maxWheelSpeed / moduleRadius,
            Math.sqrt(0.8 * (0.40 + 0.60 * share) * friction / (0.12 * moduleRadius))));
  }

  public double rotationAccel(double velocity) {
    double share = MathUtil.clamp(1 - velocity / 0.50, 0, 1);
    return Math.min(maxAlpha, (0.40 + 0.60 * share) * friction / (inertiaShare * moduleRadius));
  }
}
