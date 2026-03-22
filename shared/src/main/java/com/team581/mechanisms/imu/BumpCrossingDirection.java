package com.team581.mechanisms.imu;

import com.team581.util.FmsUtil;

public enum BumpCrossingDirection {
  TOWARDS_DRIVER_STATION,
  AWAY_FROM_DRIVER_STATION;

  /**
   * Get the X offset sign for this direction. Positive = towards red (+X), negative = towards blue
   * (-X).
   */
  public double getXSign() {
    boolean towardsRed =
        (this == TOWARDS_DRIVER_STATION && FmsUtil.isRedAlliance())
            || (this == AWAY_FROM_DRIVER_STATION && !FmsUtil.isRedAlliance());
    return towardsRed ? 1.0 : -1.0;
  }
}
