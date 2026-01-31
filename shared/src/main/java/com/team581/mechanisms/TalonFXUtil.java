package com.team581.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXUtil {
  /**
   * Get the absolute position of a {@link TalonFX}. Only works if you have not yet called {@link
   * TalonFX#setPosition(double)} on the motor.
   *
   * @return The absolute position of the motor in rotations
   */
  public static double getAbsolutePosition(TalonFX motor) {
    // Get the rotor position
    var rotorPosition = motor.getRotorPosition().getValueAsDouble();

    // Modulo the position to be within [0, 1] rotations
    return rotorPosition % 1.0;
  }

  private TalonFXUtil() {}
}
