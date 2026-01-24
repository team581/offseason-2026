package com.team581.math;

/** Modifies WPILib LinearFilter to work with angles, taking into account wrap-around */
public class CircularFilter {
  private final double[] xWindow;
  private final double[] yWindow;
  private int index = 0;
  private int count = 0;
  private double xSum = 0;
  private double ySum = 0;

  /**
   * Creates a new CircularFilter.
   *
   * @param size The number of samples to include in the moving average.
   */
  public CircularFilter(int size) {
    xWindow = new double[size];
    yWindow = new double[size];
  }

  /**
   * Calculates the next value of the filter.
   *
   * @param newAngleDegrees The new angle to add to the filter, in degrees.
   * @return The filtered angle, in degrees.
   */
  public double calculate(double newAngleDegrees) {
    double newAngleRadians = Math.toRadians(newAngleDegrees);
    double newX = Math.cos(newAngleRadians);
    double newY = Math.sin(newAngleRadians);

    if (count < xWindow.length) {
      count++;
    } else {
      xSum -= xWindow[index];
      ySum -= yWindow[index];
    }

    xSum += newX;
    ySum += newY;
    xWindow[index] = newX;
    yWindow[index] = newY;

    index = (index + 1) % xWindow.length;

    double avgX = xSum / count;
    double avgY = ySum / count;

    return Math.toDegrees(Math.atan2(avgY, avgX));
  }

  /** Fills the filter with a constant angle. */
  public void fill(double angleDegrees) {
    double angleRadians = Math.toRadians(angleDegrees);
    double x = Math.cos(angleRadians);
    double y = Math.sin(angleRadians);

    for (int i = 0; i < xWindow.length; i++) {
      xWindow[i] = x;
      yWindow[i] = y;
    }

    index = 0;
    count = xWindow.length;
    xSum = x * count;
    ySum = y * count;
  }

  /** Resets the filter. */
  public void reset() {
    index = 0;
    count = 0;
    xSum = 0;
    ySum = 0;
  }
}
