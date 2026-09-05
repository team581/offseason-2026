package frc.robot.power_manager;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(50, 80, 15, 40, 10, 10, 10, 70, 15),
  PRIORITIZE_INTAKE(50, 85, 15, 40, 10, 10, 10, 10, 15),
  IDLE(50, 50, 25, 40, 10, 10, 10, 35, 25),
  SCORING(50, 20, 15, 40, 20, 20, 20, 30, 15),
  SCORING_FAR(50, 20, 15, 40, 10, 10, 10, 20, 15),
  BEAST_MODE(50, 10, 15, 40, 30, 30, 30, 20, 15),
  FEEDING(35, 20, 15, 40, 30, 30, 30, 3, 15),
  FEEDING_FAR(35, 20, 15, 40, 15, 15, 15, 30, 15),

  TURBO_MODE(30, 5, 18, 10, 10, 10, 10, 70, 18);

  final double shooterSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;
  final double shooterHoodSupplyCurrent;
  final double feederSupplyCurrent;
  final double conveyorSupplyCurrent;
  final double funnelerSupplyCurrent;
  final double swerveSupplyCurrent;
  final double turretSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double feederSupplyCurrent,
      double conveyorSupplyCurrent,
      double funnelerSupplyCurrent,
      double swerveSupplyCurrent,
      double turretSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
    this.conveyorSupplyCurrent = conveyorSupplyCurrent;
    this.funnelerSupplyCurrent = funnelerSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
    this.turretSupplyCurrent = turretSupplyCurrent;
  }
}
