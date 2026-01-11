package com.team581;

import edu.wpi.first.wpilibj.RobotController;
import java.util.Optional;

public enum RobotKind {
  ALPHA_BOT("placeholder1"),
  COMP_BOT("placeholder2"),
  PRACTICE_BOT("placeholder3");

  /**
   * Returns the RobotKind by matching the serial number to a known RobotKind. If the serial number
   * can't be matched, returns empty.
   */
  public static Optional<RobotKind> fromSerialNumber() {
    return switch (RobotController.getSerialNumber()) {
      case "placeholder1" -> Optional.of(ALPHA_BOT);
      case "placeholder2" -> Optional.of(COMP_BOT);
      case "placeholder3" -> Optional.of(PRACTICE_BOT);
      default -> Optional.empty();
    };
  }

  public final String serialNumber;

  RobotKind(String serialNumber) {
    this.serialNumber = serialNumber;
  }
}
