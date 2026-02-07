package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;
import org.junit.jupiter.api.Test;

final class PolynomialRegressionTest {
  @Test
  void calculateReturnsCorrectValueForQuadratic() {
    // Polynomial: 1 + 2x + 3x^2
    var regression = PolynomialRegression.quadratic("Test", 1, 2, 3);

    assertEquals(1, regression.calculate(0), 1e-9);
    assertEquals(6, regression.calculate(1), 1e-9);
    assertEquals(17, regression.calculate(2), 1e-9);
    assertEquals(34, regression.calculate(3), 1e-9);
  }

  @Test
  void quadraticFromPointsFitsCorrectly() {
    // Points on y = 2 + 3x + 4x^2
    var points = Map.of(0.0, 2.0, 1.0, 9.0, 2.0, 24.0, 3.0, 47.0);

    var regression = PolynomialRegression.quadratic("Test", points);

    assertEquals(2, regression.calculate(0), 1e-6);
    assertEquals(9, regression.calculate(1), 1e-6);
    assertEquals(24, regression.calculate(2), 1e-6);
    assertEquals(47, regression.calculate(3), 1e-6);
    // Verify interpolation at a non-sample point
    // At x=1.5: 2 + 4.5 + 9 = 15.5
    assertEquals(15.5, regression.calculate(1.5), 1e-6);
  }

  @Test
  void toStringFormatsCorrectly() {
    // Polynomial: 5 + 0x + 2x^2 -> should show "2 x^2 + 5"
    var regression = PolynomialRegression.quadratic("Test", 5, 0, 2);

    assertEquals("2 x^2 + 5", regression.toString());
  }

  @Test
  void toStringHandlesNegativeCoefficients() {
    // Polynomial: -1 + -2x + 3x^2
    var regression = PolynomialRegression.quadratic("Test", -1, -2, 3);

    assertEquals("3 x^2 - 2 x - 1", regression.toString());
  }

  @Test
  void toStringHandlesZeroPolynomial() {
    var regression = PolynomialRegression.quadratic("Test", 0, 0, 0);

    assertEquals("0", regression.toString());
  }
}
