package com.team581.signals;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Central registry of CTRE {@link BaseStatusSignal}s to be refreshed in a single batch once per
 * robot loop. Subsystems should register their signals in their constructor, and {@link
 * #refreshAll()} will be called exactly once per loop before any {@code collectInputs()} runs.
 */
public final class GlobalSignals {
  private static final List<BaseStatusSignal> SIGNALS = new ArrayList<>();

  public static void refreshAll() {
    if (!SIGNALS.isEmpty()) {
      BaseStatusSignal.refreshAll(SIGNALS);
    }
  }

  public static void register(BaseStatusSignal... signals) {
    Collections.addAll(SIGNALS, signals);
  }

  private GlobalSignals() {}
}
