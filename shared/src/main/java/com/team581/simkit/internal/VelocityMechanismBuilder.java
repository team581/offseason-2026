package com.team581.simkit.internal;

import static com.google.common.base.Preconditions.checkState;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.google.errorprone.annotations.CanIgnoreReturnValue;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/** Builder for {@link VelocityMechanism}. */
public final class VelocityMechanismBuilder {
  private final List<SimMotor> motors = new ArrayList<>();
  private OptionalDouble minVelocity = OptionalDouble.empty();
  private OptionalDouble maxVelocity = OptionalDouble.empty();

  @CanIgnoreReturnValue
  public VelocityMechanismBuilder addMotor(TalonFX motor) {
    return addMotor(motor, motor.getSimState().Orientation);
  }

  @CanIgnoreReturnValue
  public VelocityMechanismBuilder addMotor(TalonFX motor, ChassisReference orientation) {
    motors.add(SimMotor.of(motor, orientation));
    return this;
  }

  public VelocityMechanism build() {
    checkState(!motors.isEmpty(), "At least one motor is required");

    return new VelocityMechanism(motors, minVelocity, maxVelocity);
  }

  @CanIgnoreReturnValue
  public VelocityMechanismBuilder withMaxVelocity(double maxVelocity) {
    this.maxVelocity = OptionalDouble.of(maxVelocity);
    return this;
  }

  @CanIgnoreReturnValue
  public VelocityMechanismBuilder withMinVelocity(double minVelocity) {
    this.minVelocity = OptionalDouble.of(minVelocity);
    return this;
  }
}
