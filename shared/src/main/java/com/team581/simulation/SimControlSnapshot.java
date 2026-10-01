package com.team581.simulation;

import java.util.List;

/** Versioned, complete input frame. Never contains NetworkTables values. */
public record SimControlSnapshot(
    int version,
    long sequence,
    boolean enabled,
    boolean estop,
    String mode,
    String alliance,
    double matchTime,
    List<Joystick> joysticks) {
  public record Joystick(
      String name, boolean xbox, List<Double> axes, List<Boolean> buttons, List<Integer> povs) {}

  public void validate() {
    if (version != 1
        || sequence < 0
        || mode == null
        || alliance == null
        || !List.of("teleop", "auto", "test").contains(mode)
        || !List.of("Unknown", "Red1", "Red2", "Red3", "Blue1", "Blue2", "Blue3").contains(alliance)
        || !Double.isFinite(matchTime)
        || matchTime < -1
        || matchTime > 3600
        || joysticks == null
        || joysticks.size() != 6) {
      throw new IllegalArgumentException("Invalid control frame");
    }
    for (var stick : joysticks) {
      if (stick == null
          || stick.name() == null
          || stick.name().length() > 128
          || stick.axes() == null
          || stick.axes().size() > 12
          || stick.buttons() == null
          || stick.buttons().size() > 32
          || stick.povs() == null
          || stick.povs().size() > 12) {
        throw new IllegalArgumentException("Invalid joystick description");
      }
      for (var axis : stick.axes()) {
        if (axis == null || !Double.isFinite(axis) || Math.abs(axis) > 1) {
          throw new IllegalArgumentException("Invalid joystick axis");
        }
      }
      for (var button : stick.buttons()) {
        if (button == null) {
          throw new IllegalArgumentException("Invalid joystick button");
        }
      }
      for (var pov : stick.povs()) {
        if (pov == null || pov < -1 || pov >= 360) {
          throw new IllegalArgumentException("Invalid joystick POV");
        }
      }
    }
  }
}
