package com.team581.util.scheduling;

import static java.util.Comparator.comparingInt;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public final class SubsystemExecutionSequencer {
  private static final Comparator<Subsystem> BY_PRIORITY_DESCENDING =
      comparingInt((Subsystem subsystem) -> subsystem.getPriority().getValue()).reversed();

  /**
   * Kept sorted by priority (highest first). A {@link java.util.PriorityQueue} cannot be used here:
   * its iterator traverses heap order, not sorted order, so periodic execution would not respect
   * priorities.
   */
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
    // Stable sort: equal priorities keep registration order
    SUBSYSTEMS.sort(BY_PRIORITY_DESCENDING);
  }

  private SubsystemExecutionSequencer() {}
}
