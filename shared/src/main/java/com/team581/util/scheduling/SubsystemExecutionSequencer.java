package com.team581.util.scheduling;

import static java.util.Comparator.comparingInt;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

public final class SubsystemExecutionSequencer {
  private static final List<Subsystem> SUBSYSTEMS = new ArrayList<>();

  public static RobotMatchState getStage() {
    if (DriverStation.isTeleopEnabled()) {
      return RobotMatchState.TELEOP;
    } else if (DriverStation.isAutonomousEnabled()) {
      return RobotMatchState.AUTONOMOUS;
    } else {
      return RobotMatchState.DISABLED;
    }
  }

  public static void periodic() {
    for (Subsystem subsystem : SUBSYSTEMS) {
      subsystem.periodic();
    }
  }

  public static void registerSubsystem(Subsystem subsystem) {
    SUBSYSTEMS.add(subsystem);
    SUBSYSTEMS.sort(
        comparingInt((Subsystem registered) -> registered.getPriority().getValue()).reversed());
  }

  private SubsystemExecutionSequencer() {}
}
