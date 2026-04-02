package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.AlsoNegation;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import com.team581.math.MathHelpers;
import edu.wpi.first.math.MathUtil;

/** Refaster rules to prefer {@link MathUtil} utility methods. */
class MathUtilRules {
  /**
   * Prefer {@link MathUtil#isNear(double, double, double, double, double)} with continuous input
   * bounds over manually wrapping with {@link MathHelpers#angleModulus(double)} inside {@link
   * MathUtil#isNear(double, double, double)}.
   */
  static class IsNearAngleModulus {
    @BeforeTemplate
    boolean before(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, MathHelpers.angleModulus(actual), tolerance);
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, actual, tolerance, -180, 180);
    }
  }

  /**
   * Prefer comparing two values directly with {@link MathUtil#isNear(double, double, double)} over
   * comparing their difference to zero.
   */
  static class IsNearZeroDifference {
    @BeforeTemplate
    boolean before(double a, double b, double tolerance) {
      return MathUtil.isNear(0, a - b, tolerance);
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement(double a, double b, double tolerance) {
      return MathUtil.isNear(a, b, tolerance);
    }
  }

  /**
   * Prefer comparing two values directly with {@link MathUtil#isNear(double, double, double,
   * double, double)} over comparing their difference to zero.
   */
  static class IsNearZeroDifferenceContinuous {
    @BeforeTemplate
    boolean before(double a, double b, double tolerance, double min, double max) {
      return MathUtil.isNear(0, a - b, tolerance, min, max);
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement(double a, double b, double tolerance, double min, double max) {
      return MathUtil.isNear(a, b, tolerance, min, max);
    }
  }

  /**
   * Prefer {@link MathUtil#isNear(double, double, double)} over manually comparing {@code
   * Math.abs(a - b) <= tolerance}.
   */
  static class MathAbsIsNear {
    @BeforeTemplate
    boolean before(double expected, double actual, double tolerance) {
      return Math.abs(expected - actual) < tolerance;
    }

    @BeforeTemplate
    boolean beforeEqual(double expected, double actual, double tolerance) {
      return Math.abs(expected - actual) <= tolerance;
    }

    @AfterTemplate
    boolean replacement(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, actual, tolerance);
    }
  }

  /**
   * Prefer {@link MathUtil#isNear(double, double, double)} over manually comparing {@code tolerance
   * >= Math.abs(a - b)}.
   */
  static class MathAbsIsNearReversed {
    @BeforeTemplate
    boolean before(double expected, double actual, double tolerance) {
      return tolerance >= Math.abs(expected - actual);
    }

    @BeforeTemplate
    boolean beforeEqual(double expected, double actual, double tolerance) {
      return tolerance >= Math.abs(expected - actual);
    }

    @AfterTemplate
    boolean replacement(double expected, double actual, double tolerance) {
      return MathUtil.isNear(expected, actual, tolerance);
    }
  }

  /**
   * Prefer {@link MathUtil#isNear(double, double, double)} over manually comparing {@code
   * Math.abs(x) < tolerance}.
   */
  static class MathAbsIsNearZero {
    @BeforeTemplate
    boolean before(double x, double tolerance) {
      return Math.abs(x) < tolerance;
    }

    @BeforeTemplate
    boolean beforeEqual(double x, double tolerance) {
      return Math.abs(x) <= tolerance;
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement(double x, double tolerance) {
      return MathUtil.isNear(0, x, tolerance);
    }
  }

  private MathUtilRules() {}
}
