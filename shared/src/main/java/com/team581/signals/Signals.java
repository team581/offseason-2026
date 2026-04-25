package com.team581.signals;

import com.ctre.phoenix6.StatusSignalCollection;

/**
 * Central registry of CTRE status signals to be refreshed in a single batch once per robot loop.
 * Subsystems should add their signals to {@link #ALL} in their constructor, and {@link
 * StatusSignalCollection#refreshAll()} will be called exactly once per loop before any {@code
 * collectInputs()} runs.
 */
public final class Signals {
  public static final StatusSignalCollection ALL = new StatusSignalCollection();

  private Signals() {}
}
