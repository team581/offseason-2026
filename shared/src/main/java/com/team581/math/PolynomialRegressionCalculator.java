package com.team581.math;

public class PolynomialRegressionCalculator {

  public static double polynomialRegression(
      double x, double leadingCoefficient, double slope, double yInt) {
    return (leadingCoefficient * (x * x)) + (slope * x) + yInt;
  }
}
