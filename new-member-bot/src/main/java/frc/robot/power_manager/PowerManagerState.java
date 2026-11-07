package frc.robot.power_manager;

import static frc.robot.Hardware.SUPPLY_CURRENT_LIMIT_SCALE;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(50, 80, 15, 40, 10, 10, 70),
  PRIORITIZE_INTAKE(50, 60, 15, 40, 10, 10, 30),
  IDLE(50, 40, 25, 40, 10, 10, 35),
  SCORING(50, 10, 15, 40, 20, 15, 30),
  SCORING_FAR(50, 10, 15, 40, 15, 15, 20),
  BEAST_MODE(50, 10, 15, 40, 15, 15, 20),
  FEEDING(35, 10, 15, 40, 20, 15, 35),
  FEEDING_FAR(35, 10, 15, 40, 15, 15, 30),

  TURBO_MODE(30, 5, 18, 10, 10, 10, 70);

  final double shooterSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;
  final double shooterHoodSupplyCurrent;
  final double feederSupplyCurrent;
  final double conveyorSupplyCurrent;
  final double swerveSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double feederSupplyCurrent,
      double conveyorSupplyCurrent,
      double swerveSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.intakeSupplyCurrent = intakeSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.deploySupplyCurrent = deploySupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.feederSupplyCurrent = feederSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.conveyorSupplyCurrent = conveyorSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
    this.swerveSupplyCurrent = swerveSupplyCurrent * SUPPLY_CURRENT_LIMIT_SCALE;
  }
}
