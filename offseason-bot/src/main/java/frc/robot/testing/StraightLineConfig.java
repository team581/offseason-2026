package frc.robot.testing;

import java.util.Map;
import java.util.function.ToDoubleFunction;

/** SI units except direction and heading tolerance, which are degrees. */
public record StraightLineConfig(
    double distance,
    double direction,
    double maxVelocity,
    double maxAcceleration,
    double timeout,
    double positionTolerance,
    double headingTolerance,
    double stoppedVelocity,
    double settleSeconds,
    double crossTrackTolerance) {
  public static final Map<String, Double> DEFAULTS =
      Map.ofEntries(
          Map.entry("distance", 2.0),
          Map.entry("direction", 0.0),
          Map.entry("maxVelocity", 1.0),
          Map.entry("maxAcceleration", 0.75),
          Map.entry("timeout", 10.0),
          Map.entry("positionTolerance", 0.10),
          Map.entry("headingTolerance", 3.0),
          Map.entry("stoppedVelocity", 0.10),
          Map.entry("settleSeconds", 0.25),
          Map.entry("crossTrackTolerance", 0.20));

  public static StraightLineConfig from(ToDoubleFunction<String> value) {
    return new StraightLineConfig(
        value.applyAsDouble("distance"),
        value.applyAsDouble("direction"),
        value.applyAsDouble("maxVelocity"),
        value.applyAsDouble("maxAcceleration"),
        value.applyAsDouble("timeout"),
        value.applyAsDouble("positionTolerance"),
        value.applyAsDouble("headingTolerance"),
        value.applyAsDouble("stoppedVelocity"),
        value.applyAsDouble("settleSeconds"),
        value.applyAsDouble("crossTrackTolerance"));
  }

  public StraightLineConfig {
    if (!Double.isFinite(distance) || distance == 0 || !Double.isFinite(direction)) {
      throw new IllegalArgumentException(
          "Distance must be finite and nonzero; direction must be finite");
    }
    for (double positive :
        new double[] {
          maxVelocity,
          maxAcceleration,
          timeout,
          positionTolerance,
          headingTolerance,
          stoppedVelocity,
          settleSeconds,
          crossTrackTolerance
        }) {
      if (!Double.isFinite(positive) || positive <= 0) {
        throw new IllegalArgumentException(
            "Limits, tolerances and durations must be finite and positive");
      }
    }
    if (positionTolerance >= Math.abs(distance) || settleSeconds >= timeout) {
      throw new IllegalArgumentException(
          "Position tolerance must be smaller than distance; settle time smaller than timeout");
    }
  }
}
