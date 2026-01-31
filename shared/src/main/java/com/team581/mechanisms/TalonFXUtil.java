package com.team581.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXUtil {
  /**
   * Get the absolute position of a {@link TalonFX}. Only works if you have not yet called {@link
   * TalonFX#setPosition(double)} on the motor.
   */
  public static double getAbsolutePosition(TalonFX motor) {
    // Get the rotor position
    var rotorPosition = motor.getPosition().getValueAsDouble();

    // Modulo the position to be within [0, 1] rotations
    var absolutePosition = rotorPosition % 1.0;

    // Return the absolute position
    return absolutePosition;
  }

  private TalonFXUtil() {}
}
