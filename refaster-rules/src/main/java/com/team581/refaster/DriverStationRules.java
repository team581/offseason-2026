package com.team581.refaster;

import com.google.errorprone.refaster.annotation.AfterTemplate;
import com.google.errorprone.refaster.annotation.AlsoNegation;
import com.google.errorprone.refaster.annotation.BeforeTemplate;
import edu.wpi.first.wpilibj.DriverStation;

/** Refaster rules to prefer {@link DriverStation} combined mode-and-enabled checks. */
class DriverStationRules {
  /**
   * Prefer {@link DriverStation#isAutonomousEnabled()} over manually combining {@link
   * DriverStation#isEnabled()} and {@link DriverStation#isAutonomous()}.
   */
  static class IsAutonomousEnabled {
    @BeforeTemplate
    boolean before() {
      return DriverStation.isEnabled() && DriverStation.isAutonomous();
    }

    @BeforeTemplate
    boolean beforeReversed() {
      return DriverStation.isAutonomous() && DriverStation.isEnabled();
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement() {
      return DriverStation.isAutonomousEnabled();
    }
  }

  /**
   * Prefer {@link DriverStation#isTeleopEnabled()} over manually combining {@link
   * DriverStation#isEnabled()} and {@link DriverStation#isTeleop()}.
   */
  static class IsTeleopEnabled {
    @BeforeTemplate
    boolean before() {
      return DriverStation.isEnabled() && DriverStation.isTeleop();
    }

    @BeforeTemplate
    boolean beforeReversed() {
      return DriverStation.isTeleop() && DriverStation.isEnabled();
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement() {
      return DriverStation.isTeleopEnabled();
    }
  }

  /**
   * Prefer {@link DriverStation#isTestEnabled()} over manually combining {@link
   * DriverStation#isEnabled()} and {@link DriverStation#isTest()}.
   */
  static class IsTestEnabled {
    @BeforeTemplate
    boolean before() {
      return DriverStation.isEnabled() && DriverStation.isTest();
    }

    @BeforeTemplate
    boolean beforeReversed() {
      return DriverStation.isTest() && DriverStation.isEnabled();
    }

    @AfterTemplate
    @AlsoNegation
    boolean replacement() {
      return DriverStation.isTestEnabled();
    }
  }

  private DriverStationRules() {}
}
