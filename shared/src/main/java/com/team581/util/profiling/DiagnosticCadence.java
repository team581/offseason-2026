package com.team581.util.profiling;

/** Controls the publication rate of non-control diagnostic telemetry. */
public final class DiagnosticCadence {
  private static boolean enabled;
  private static long loop;

  /** Advances the cadence clock once at the beginning of a robot loop. */
  public static void beginLoop() {
    loop++;
  }

  /** Enables cadence limiting. Disabled by default so other robot projects are unchanged. */
  public static void setEnabled(boolean shouldEnable) {
    enabled = shouldEnable;
    loop = 0;
  }

  /** Returns true at 5 Hz when enabled, and every call when disabled. */
  public static boolean shouldLogHeavy() {
    return !enabled || loop % 10 == 1;
  }

  /** Returns true at 10 Hz when enabled, and every call when disabled. */
  public static boolean shouldLogRoutine() {
    return !enabled || loop % 5 == 1;
  }

  private DiagnosticCadence() {}
}
