package com.team581.simkit.internal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team581.math.SlewRateLimiterStateless;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.OptionalDouble;

/** Predicts a simple velocity-controlled mechanism using TalonFX configuration state. */
public final class VelocityMechanism {
  private static double getMechanismAcceleration(List<SimMotor> devices) {
    // Collect configs for averaging
    var configs =
        devices.stream()
            .map(
                motor -> {
                  var config = new TalonFXConfiguration();
                  motor.motor().getConfigurator().refresh(config);
                  return config;
                })
            .toList();

    return configs.stream()
        .mapToDouble(c -> c.MotionMagic.MotionMagicAcceleration)
        .average()
        .orElse(0.0);
  }

  private static double desiredMechanismVelocity(List<SimMotor> devices) {
    return devices.stream()
        .mapToDouble(device -> device.motor().getClosedLoopReference().getValueAsDouble())
        .average()
        .orElse(0.0);
  }

  private final List<SimMotor> devices;
  private final OptionalDouble minVelocity;
  private final OptionalDouble maxVelocity;
  private double currentVelocity = 0.0;
  private double previousTimestamp = MathSharedStore.getTimestamp();

  /** Recomputes the predicted velocity and pushes the result into each motor sim. */
  public void update() {
    var acceleration = getMechanismAcceleration(devices);
    // When disabled, target 0 velocity to simulate spin-down from lack of voltage
    var wantedVelocity = DriverStation.isDisabled() ? 0.0 : desiredMechanismVelocity(devices);
    var boundedWantedVelocity = applyVelocityBounds(wantedVelocity);
    var newVelocity =
        SlewRateLimiterStateless.calculate(
            boundedWantedVelocity, currentVelocity, previousTimestamp, acceleration, -acceleration);

    currentVelocity = newVelocity;
    previousTimestamp = MathSharedStore.getTimestamp();

    for (var motor : devices) {
      motor.applyVelocity(newVelocity);
    }
  }

  VelocityMechanism(List<SimMotor> motors, OptionalDouble minVelocity, OptionalDouble maxVelocity) {
    this.devices = motors;
    this.minVelocity = minVelocity;
    this.maxVelocity = maxVelocity;
  }

  private double applyVelocityBounds(double velocity) {
    return MathUtil.clamp(
        velocity,
        minVelocity.orElse(Double.NEGATIVE_INFINITY),
        maxVelocity.orElse(Double.POSITIVE_INFINITY));
  }
}
