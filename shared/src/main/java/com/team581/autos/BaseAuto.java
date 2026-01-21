package com.team581.autos;

import edu.wpi.first.wpilibj.DriverStation;

public interface BaseAuto {
  Point getStartingPoint();

  /** Returns the name of this auto. */
  default String name() {
    var className = this.getClass().getSimpleName();
    return className.substring(className.lastIndexOf('.') + 1);
  }

  /**
   * Returns whether the auto should currently be scheduled when it's selected.
   *
   * <p>Defaults to run when autonomous mode is selected (including while disabled).
   *
   * <p>Can override to support running autos during teleop.
   */
  default boolean shouldRun() {
    return DriverStation.isAutonomous();
  }
}
