package com.team581.trailblazer3.config;

import static com.google.common.base.Preconditions.checkArgument;

import com.team581.trailblazer3.math.DrivetrainLimits;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;

/** Calibration, tracking response, and robot-derived limits for robot code and previews. */
public final class TrailblazerConfig {
  // Tune these on carpet
  public static final EmpiricalDriveCalibration COMP =
      new EmpiricalDriveCalibration(8.5, 6.0, 0.055, 8, 0.4);

  public static final EmpiricalDriveCalibration OFFSEASON =
      new EmpiricalDriveCalibration(8.5, 6.0, 0.055, 8, 0.4);

  public static final EmpiricalDriveCalibration NEW_MEMBER =
      new EmpiricalDriveCalibration(8.0, 5.5, 0.060, 8, 0.4);

  /**
   * Response-time multiplier for lag-derived heading P. Existing design choice, not a robot
   * measurement.
   */
  public static final double HEADING_RESPONSE_MULTIPLIER = 3;

  /**
   * Response-time multiplier for translation correction. Existing design choice; P is inverse
   * response time.
   */
  public static final double TRANSLATION_RESPONSE_MULTIPLIER = 4;

  /** Minimum translation response time in seconds; limits correction gain to 1 / this value. */
  public static final double MIN_TRANSLATION_RESPONSE_SECONDS = 0.12;

  public static DrivetrainLimits deriveLimits(
      RobotDrivetrainSpec robot, EmpiricalDriveCalibration calibration, double nominalDt) {
    var modulePositions = robot.modulePositionsMeters();
    checkArgument(!modulePositions.isEmpty(), "At least one module position is required");
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
    double wheelSpeed = 0.91 * free;
    double friction = calibration.frictionLimit();
    double lag = calibration.driveResponseLagSeconds();
    double raw = motorAcceleration(robot, free, 0) / 0.90;
    double steerEffectiveVoltage =
        Math.max(
            0,
            robot.steerMotor().nominalVoltageVolts
                - Math.min(
                    robot.staticFrictionVolts(), 0.5 * robot.steerMotor().nominalVoltageVolts));
    return new DrivetrainLimits(
        robot,
        wheelSpeed,
        wheelSpeed,
        free,
        calibration.maxBrakingDeceleration(),
        friction,
        0.85 * friction,
        0.85 * wheelSpeed / radius,
        0.85 * robot.massKg() * Math.min(raw, friction) * radius / robot.yawMoiKgMetersSq(),
        Math.min(
            0.5
                * steerEffectiveVoltage
                * robot.steerMotor().KvRadPerSecPerVolt
                / robot.steerGearRatio(),
            0.50 / (lag + nominalDt / 2)),
        lag,
        radius,
        Math.min(1, 4 * robot.yawMoiKgMetersSq() / (robot.massKg() * squaredRadii)),
        Double.POSITIVE_INFINITY,
        calibration.headingP(),
        calibration.headingD());
  }

  /**
   * Calculates limits from robot specs and carpet measurements, then applies segment caps. The
   * 0.91, 0.85, 0.90, and 0.50 factors are existing model margins, not measured robot properties.
   * See {@link DrivetrainLimits} for each output's formula and units.
   */
  public static DrivetrainLimits deriveLimits(
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      MotionLimits caps) {
    return deriveLimits(robot, calibration, nominalDt).withCaps(caps);
  }

  /** Heading P from a three-response-time correction, capped by the configured feedback gain. */
  public static double headingGain(double headingP, double lagSeconds, double periodSeconds) {
    return Math.min(headingP, 1 / (HEADING_RESPONSE_MULTIPLIER * (lagSeconds + periodSeconds / 2)));
  }

  /**
   * Acceleration in m/s² from torque / (mass * wheel radius), after gearing and modeled losses.
   * Current uses back EMF, slip current, 0.018 Ω battery resistance, and 0.95 inverter efficiency;
   * the two 0.90 factors retain the existing acceleration reserve and drivetrain efficiency.
   */
  public static double motorAcceleration(
      RobotDrivetrainSpec robot, double freeSpeed, double speed) {
    var motor = robot.driveMotor();
    double voltage = motor.nominalVoltageVolts;
    double emf =
        robot.staticFrictionVolts() + (voltage - robot.staticFrictionVolts()) * speed / freeSpeed;
    double resistance = motor.rOhms;
    double batteryResistance = 0.018;
    double inverterEfficiency = 0.95;
    double current =
        Math.min(
            robot.slipCurrentAmps(),
            Math.max(0, voltage - emf) / (resistance + 4 * batteryResistance / inverterEfficiency));
    return 0.90
        * 4
        * 0.90
        * robot.driveGearRatio()
        * motor.KtNMPerAmp
        * Math.max(0, current - motor.freeCurrentAmps)
        / (robot.massKg() * robot.wheelRadiusMeters());
  }

  /** Translation response in seconds; its inverse is the position-error gain. */
  public static double translationResponseSeconds(double lagSeconds, double periodSeconds) {
    return Math.max(
        MIN_TRANSLATION_RESPONSE_SECONDS,
        TRANSLATION_RESPONSE_MULTIPLIER * (lagSeconds + periodSeconds / 2));
  }

  private TrailblazerConfig() {}
}
