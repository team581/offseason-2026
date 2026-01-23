package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.math.util.Units;

/**
 * Refaster rules to prefer standard Java {@link Math} methods over {@link Units} conversion
 * methods.
 */
class UnitsRules {
  private UnitsRules() {}

  /** Prefer {@link Math#toRadians(double)} over {@link Units#degreesToRadians(double)}. */
  static class PreferMathToRadians {
    @BeforeTemplate
    double before(double degrees) {
      return Units.degreesToRadians(degrees);
    }

    @AfterTemplate
    double after(double degrees) {
      return Math.toRadians(degrees);
    }
  }

  /** Prefer {@link Math#toDegrees(double)} over {@link Units#radiansToDegrees(double)}. */
  static class PreferMathToDegrees {
    @BeforeTemplate
    double before(double radians) {
      return Units.radiansToDegrees(radians);
    }

    @AfterTemplate
    double after(double radians) {
      return Math.toDegrees(radians);
    }
  }
}
