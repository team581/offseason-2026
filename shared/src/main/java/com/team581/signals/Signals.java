package com.team581.signals;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.HashMap;
import java.util.Map;

/**
 * Central registry of CTRE status signals to be refreshed in a single batch once per robot loop.
 * Signals must be grouped by CAN bus, since {@link StatusSignalCollection#refreshAll()} requires
 * all signals in a collection to share a network. Subsystems should call {@link #forBus} in their
 * constructor to obtain the collection for their device's bus, and {@link #refreshAll()} will
 * refresh every per-bus collection exactly once per loop before any {@code collectInputs()} runs.
 */
public final class Signals {
  private static final Map<String, StatusSignalCollection> COLLECTIONS = new HashMap<>();

  /** Returns the collection of signals for the given CAN bus, creating one if needed. */
  public static StatusSignalCollection forBus(CANBus canBus) {
    return forBus(canBus.getName());
  }

  /** Returns the collection of signals for the given CAN bus, creating one if needed. */
  public static StatusSignalCollection forBus(String canBusName) {
    return COLLECTIONS.computeIfAbsent(canBusName, name -> new StatusSignalCollection());
  }

  /** Returns the collection of signals for the bus the given device is on. */
  public static StatusSignalCollection forDevice(ParentDevice device) {
    return forBus(device.getNetwork());
  }

  /** Refreshes every per-bus signal collection. */
  public static void refreshAll() {
    for (var collection : COLLECTIONS.values()) {
      collection.refreshAll();
    }
  }

  private Signals() {}
}
