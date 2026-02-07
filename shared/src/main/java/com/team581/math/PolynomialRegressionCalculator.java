package com.team581.math;

public class PolynomialRegressionCalculator {

  private double leadingCoefficient;
  private double slope;
  private double yInt;

  public PolynomialRegressionCalculator(double leadingCoefficient, double slope, double yInt) {
    this.leadingCoefficient = leadingCoefficient;
    this.slope = slope;
    this.yInt = yInt;
  }

  public double calculate(double x) {
    return (leadingCoefficient * (x * x)) + (slope * x) + yInt;
  }
}
