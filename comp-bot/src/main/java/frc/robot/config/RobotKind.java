package frc.robot.config;

import edu.wpi.first.wpilibj.RobotController;

public enum RobotKind {
  COMP_BOT("placeholder1"),
  PRACTICE_BOT("placeholder2");

  /** Returns the RobotKind by matching the serial number to a known RobotKind. */
  public static RobotKind fromSerialNumber() {
    return switch (RobotController.getSerialNumber()) {
      case "placeholder2" -> PRACTICE_BOT;
      default -> COMP_BOT;
    };
  }

  public final String serialNumber;

  RobotKind(String serialNumber) {
    this.serialNumber = serialNumber;
  }
}
