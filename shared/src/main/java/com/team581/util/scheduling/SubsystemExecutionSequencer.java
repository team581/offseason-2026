package com.team581.util.scheduling;

import static java.util.Comparator.comparingInt;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.PriorityQueue;
import java.util.Queue;

public final class SubsystemExecutionSequencer {
  private static final Queue<Subsystem> SUBSYSTEMS =
      new PriorityQueue<>(
          comparingInt((Subsystem subsystem) -> subsystem.getPriority().getValue()).reversed());

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
  }

  private SubsystemExecutionSequencer() {}
}
