package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.math.geometry.Translation2d;

/** Refaster rules to prefer {@link Translation2d} constants over factory methods for common values. */
class Translation2dRules {
  private Translation2dRules() {}

  /** Prefer {@link Translation2d#kZero} over equivalent constructors. */
  static class Translation2dZero {
    @BeforeTemplate
    Translation2d constructorWithArgs() {
      return new Translation2d(0.0, 0.0);
    }

    @BeforeTemplate
    Translation2d constructorNoArg() {
      return new Translation2d();
    }

    @AfterTemplate
    Translation2d after() {
      return Translation2d.kZero;
    }
  }
}
