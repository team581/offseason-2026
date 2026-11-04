package frc.robot.power_manager;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum PowerManagerState {
  AUTO_FIRST_SEGMENT(50, 80, 15, 40, 10, 10, 10, 70, 15),
  PRIORITIZE_INTAKE(50, 85, 15, 40, 10, 10, 10, 10, 15),
  IDLE(50, 50, 25, 40, 10, 10, 10, 35, 25),
  SCORING(50, 20, 15, 40, 20, 20, 20, 30, 15),
  SCORING_FAR(50, 20, 15, 40, 10, 10, 10, 20, 15),
  BEAST_MODE(50, 10, 15, 40, 30, 30, 30, 20, 15),
  FEEDING(35, 20, 15, 40, 30, 30, 30, 30, 15),
  FEEDING_FAR(35, 20, 15, 40, 15, 15, 15, 30, 15),

  TURBO_MODE(30, 5, 18, 10, 10, 10, 10, 70, 18);

  final DoubleSubscriber shooterSupplyCurrent;
  final DoubleSubscriber intakeSupplyCurrent;
  final DoubleSubscriber deploySupplyCurrent;
  final DoubleSubscriber shooterHoodSupplyCurrent;
  final DoubleSubscriber feederSupplyCurrent;
  final DoubleSubscriber conveyorSupplyCurrent;
  final DoubleSubscriber funnelerSupplyCurrent;
  final DoubleSubscriber swerveSupplyCurrent;
  final DoubleSubscriber turretSupplyCurrent;

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
    this.shooterSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/ShooterSupplyCurrentAmps",
            shooterSupplyCurrent,
            "A");
    this.intakeSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/IntakeSupplyCurrentAmps", intakeSupplyCurrent, "A");
    this.deploySupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/DeploySupplyCurrentAmps", deploySupplyCurrent, "A");
    this.shooterHoodSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/ShooterHoodSupplyCurrentAmps",
            shooterHoodSupplyCurrent,
            "A");
    this.feederSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/FeederSupplyCurrentAmps", feederSupplyCurrent, "A");
    this.conveyorSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/ConveyorSupplyCurrentAmps",
            conveyorSupplyCurrent,
            "A");
    this.funnelerSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/FunnelerSupplyCurrentAmps",
            funnelerSupplyCurrent,
            "A");
    this.swerveSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/SwerveSupplyCurrentAmps", swerveSupplyCurrent, "A");
    this.turretSupplyCurrent =
        DogLog.tunable(
            "PowerManager/States/" + name() + "/TurretSupplyCurrentAmps", turretSupplyCurrent, "A");
  }
}
