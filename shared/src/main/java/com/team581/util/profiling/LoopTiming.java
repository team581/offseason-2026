package com.team581.util.profiling;

import dev.doglog.DogLog;

/** Low-overhead, opt-in timing for the main robot thread. */
public final class LoopTiming {
  private static boolean enabled;

  public static void end(String key, long startNanos) {
    if (enabled) {
      DogLog.log(key, (System.nanoTime() - startNanos) / 1_000_000_000.0);
    }
  }

  public static boolean isEnabled() {
    return enabled;
  }

  public static void recordClockOverhead() {
    if (enabled) {
      long startNanos = System.nanoTime();
      long endNanos = System.nanoTime();
      DogLog.log("Scheduler/ProfilerClockOverhead", (endNanos - startNanos) / 1_000_000_000.0);
    }
  }

  public static void setEnabled(boolean shouldEnable) {
    enabled = shouldEnable;
  }

  public static long start() {
    return enabled ? System.nanoTime() : 0L;
  }

  private LoopTiming() {}
}
