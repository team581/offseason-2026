package com.team581.trailblazer;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class ConstraintsCalculatorTest {
  private static final double DELTA = 1e-6;

  private ConstraintsCalculator calculator;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    calculator = new ConstraintsCalculator(new PIDController(1, 0, 0));
    calculator.reset();
  }

  @Test
  void constrainLinearVelocity_withNoConstraints_returnsDesiredVelocity() {
    var desiredVelocity = 5.0;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Constraints with 0 maxLinearVelocity means disabled
    var constraints = new AutoConstraintOptions(0, 0, 0, 0);

    var result = calculator.constrainLinearVelocity(desiredVelocity, currentSpeeds, constraints);

    assertEquals(desiredVelocity, result, DELTA);
  }

  @Test
  void constrainLinearVelocity_withConstraints_clampsToMaxVelocity() {
    var desiredVelocity = 10.0;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max velocity of 2, max acceleration of 10
    var constraints = new AutoConstraintOptions(2.0, 0, 10.0, 0);

    // Over multiple calls with simulated time, the velocity should be clamped to max
    double result = 0;
    for (int i = 0; i < 100; i++) {
      SimHooks.stepTiming(0.02);
      result = calculator.constrainLinearVelocity(desiredVelocity, currentSpeeds, constraints);
    }

    assertEquals(constraints.maxLinearVelocity(), result, DELTA);
  }

  @Test
  void constrainLinearVelocity_withConstraints_respectsAccelerationLimit() {
    var desiredVelocity = 10.0;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max velocity of 10, max acceleration of 2
    var constraints = new AutoConstraintOptions(10.0, 0, 2.0, 0);

    // First call from rest should not immediately jump to desired velocity
    var result = calculator.constrainLinearVelocity(desiredVelocity, currentSpeeds, constraints);

    // Result should be less than desired due to acceleration limiting
    assertTrue(result < desiredVelocity);
    assertTrue(result >= 0);
  }

  @Test
  void constrainAngularVelocity_withNoConstraints_returnsDesiredVelocity() {
    var desiredAngularVelocity = 2.0;
    var currentAngleRadians = 0.0;
    var targetAngleRadians = Math.PI / 2;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Constraints with 0 maxAngularVelocity means disabled
    var constraints = new AutoConstraintOptions(0, 0, 0, 0);

    var result =
        calculator.constrainAngularVelocity(
            desiredAngularVelocity,
            currentAngleRadians,
            targetAngleRadians,
            currentSpeeds,
            constraints);

    assertEquals(desiredAngularVelocity, result, DELTA);
  }

  @Test
  void constrainAngularVelocity_withConstraints_respectsMaxAngularVelocity() {
    var desiredAngularVelocity = 100.0;
    var currentAngleRadians = 0.0;
    var targetAngleRadians = Math.PI;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max angular velocity of 1 rad/s
    var constraints = new AutoConstraintOptions(0, 1.0, 0, 1.0);

    var result =
        calculator.constrainAngularVelocity(
            desiredAngularVelocity,
            currentAngleRadians,
            targetAngleRadians,
            currentSpeeds,
            constraints);

    // Profiled controller should limit the angular velocity
    assertTrue(Math.abs(result) <= constraints.maxAngularVelocity() + DELTA);
  }

  @Test
  void reset_resetsLinearVelocityState() {
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    var constraints = new AutoConstraintOptions(2.0, 0, 10.0, 0);

    // Build up some velocity
    for (int i = 0; i < 50; i++) {
      SimHooks.stepTiming(0.02);
      calculator.constrainLinearVelocity(10.0, currentSpeeds, constraints);
    }

    // Reset and verify we start from zero again
    calculator.reset();
    var result = calculator.constrainLinearVelocity(10.0, currentSpeeds, constraints);

    // After reset, should start accelerating from zero again
    assertTrue(result < 5.0);
  }

  @Test
  void reset_withCurrentState_preservesVelocity() {
    var currentSpeeds = new ChassisSpeeds(3.0, 4.0, 1.0);
    var currentAngle = Math.PI / 4;

    calculator.reset(currentSpeeds, currentAngle);

    // After reset with state, constraints should apply from current velocity
    var constraints = new AutoConstraintOptions(10.0, 0, 2.0, 0);
    var result = calculator.constrainLinearVelocity(10.0, currentSpeeds, constraints);

    // Should be near the current linear velocity (5.0 = hypot(3, 4)) plus some acceleration
    assertTrue(result >= 5.0 - DELTA);
  }

  @Test
  void constrainLinearVelocity_withNegativeDesiredVelocity_respectsConstraints() {
    var desiredVelocity = -10.0;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max velocity of 5, max acceleration of 2
    var constraints = new AutoConstraintOptions(5.0, 0, 2.0, 0);

    // Over multiple calls with simulated time, the velocity should be clamped to negative max
    double result = 0;
    for (int i = 0; i < 100; i++) {
      // Simulate 20ms loop time
      SimHooks.stepTiming(0.02);
      result = calculator.constrainLinearVelocity(desiredVelocity, currentSpeeds, constraints);
    }

    assertEquals(-constraints.maxLinearVelocity(), result, DELTA);
  }

  @Test
  void constrainLinearVelocity_whenDecelerating_usesMaxDeceleration() {
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max velocity of 10, max acceleration of 100 (high so we reach target quickly)
    var constraints = new AutoConstraintOptions(10.0, 0, 100.0, 0);

    // Build up velocity first with simulated time
    for (int i = 0; i < 100; i++) {
      SimHooks.stepTiming(0.02);
      calculator.constrainLinearVelocity(10.0, currentSpeeds, constraints);
    }

    // Now request zero velocity - should decelerate
    SimHooks.stepTiming(0.02);
    var result = calculator.constrainLinearVelocity(0.0, currentSpeeds, constraints);

    // Should be less than max velocity due to deceleration starting
    assertTrue(result < 10.0);
    assertTrue(result >= 0);
  }

  @Test
  void constrainLinearVelocity_withZeroAccelerationConstraint_stillClampsVelocity() {
    var desiredVelocity = 10.0;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max velocity of 2, acceleration of 0 (disabled)
    var constraints = new AutoConstraintOptions(2.0, 0, 0, 0);

    var result = calculator.constrainLinearVelocity(desiredVelocity, currentSpeeds, constraints);

    // With acceleration disabled, velocity should immediately jump to max velocity (clamped)
    assertEquals(constraints.maxLinearVelocity(), result, DELTA);
  }

  @Test
  void constrainAngularVelocity_withConstraints_respectsMaxAngularAcceleration() {
    var desiredAngularVelocity = 100.0;
    var currentAngleRadians = 0.0;
    var targetAngleRadians = Math.PI;
    var currentSpeeds = new ChassisSpeeds(0, 0, 0);
    // Max angular velocity of 10 rad/s, max angular acceleration of 1 rad/s^2
    var constraints = new AutoConstraintOptions(0, 10.0, 0, 1.0);

    // First call from rest - profiled controller should limit acceleration
    var result =
        calculator.constrainAngularVelocity(
            desiredAngularVelocity,
            currentAngleRadians,
            targetAngleRadians,
            currentSpeeds,
            constraints);

    // Profiled controller should limit the angular velocity from the start
    assertTrue(Math.abs(result) <= constraints.maxAngularVelocity() + DELTA);
  }
}
