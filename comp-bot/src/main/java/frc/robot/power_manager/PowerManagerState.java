package frc.robot.power_manager;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(10, 100, 100, 30, 10, 60, 18),
  IDLE(100, 50, 18, 10, 50, 50, 40),
  SHOOTING(100, 50, 18, 10, 50, 50, 10);

  final double shooterHoodSupplyCurrent;
  final double swerveSupplyCurrent;
  final double shooterSupplyCurrent;
  final double intakeSupplyCurrent;
  final double deploySupplyCurrent;
  final double feederSupplyCurrent;
  final double conveyorSupplyCurrent;

  PowerManagerState(
      double shooterSupplyCurrent,
      double intakeSupplyCurrent,
      double deploySupplyCurrent,
      double shooterHoodSupplyCurrent,
      double feederSupplyCurrent,
      double conveyorSupplyCurrent,
      double swerveSupplyCurrent) {
    this.shooterSupplyCurrent = shooterSupplyCurrent;
    this.intakeSupplyCurrent = intakeSupplyCurrent;
    this.deploySupplyCurrent = deploySupplyCurrent;
    this.shooterHoodSupplyCurrent = shooterHoodSupplyCurrent;
    this.feederSupplyCurrent = feederSupplyCurrent;
    this.conveyorSupplyCurrent = conveyorSupplyCurrent;
    this.swerveSupplyCurrent = swerveSupplyCurrent;
  }
}
