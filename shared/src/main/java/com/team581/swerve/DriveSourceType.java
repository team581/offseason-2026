package com.team581.swerve;

/** Describes how the drive source should be interpreted. */
public enum DriveSourceType {
  /**
   * This drive source outputs velocities that should be followed open loop. They are field-relative
   * from the perspective of the driver.
   */
  DRIVER_PERSPECTIVE_OPEN_LOOP,
  /**
   * This drive source outputs velocities that should be followed closed loop. They are totally
   * field-relative, regardless of alliance.
   */
  FIELD_CENTRIC_CLOSED_LOOP;
}
