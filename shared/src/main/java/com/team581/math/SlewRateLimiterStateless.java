package com.team581.math;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A stateless slew rate limiter that limits the rate of change of an input value.
 *
 * <p>Unlike WPILib's {@link edu.wpi.first.math.filter.SlewRateLimiter}, this implementation does
 * not store state internally. Instead, state is passed in as parameters, allowing dynamic rate
 * limits without creating new objects.
 */
public final class SlewRateLimiterStateless {
  private SlewRateLimiterStateless() {}

  /**
   * Calculates the rate-limited value.
   *
   * @param input The input value to be rate-limited.
   * @param currentValue The current output value.
   * @param previousTimestamp The timestamp of the previous value.
   * @param positiveRateLimit The maximum rate of increase (units per second). Must be positive.
   * @param negativeRateLimit The maximum rate of decrease (units per second). Must be negative.
   * @return The rate-limited output value.
   */
  public static double calculate(
      double input,
      double currentValue,
      double previousTimestamp,
      double positiveRateLimit,
      double negativeRateLimit) {
    var currentTime = MathSharedStore.getTimestamp();
    var elapsedTime = currentTime - previousTimestamp;
    currentValue +=
        MathUtil.clamp(
            input - currentValue, negativeRateLimit * elapsedTime, positiveRateLimit * elapsedTime);
    return currentValue;
  }
}
