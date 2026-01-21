package com.team581.util.state_machines;

import static java.util.Comparator.comparingInt;

import com.team581.util.scheduling.RegisteredSubsystem;
import com.team581.util.scheduling.SubsystemPriorityBase;
import java.util.PriorityQueue;
import java.util.Queue;

/** Helps ensure that state machines can collect inputs before executing state actions. */
public class StateMachineSubsystemInputManager extends RegisteredSubsystem {
  // Sort by lowest priority first
  private final Queue<StateMachineSubsystem<?>> stateMachineSubsystems =
      new PriorityQueue<>(comparingInt(stateMachine -> stateMachine.getPriority().getValue()));

  @Override
  public SubsystemPriorityBase getPriority() {
    // Ensures that state machine inputs are gathered at the right time
    // Subsystem inputs are collected in reverse order of priority (so lowest priority first)
    return () -> 999;
  }

  @Override
  public void periodic() {
    for (var stateMachineSubsystem : stateMachineSubsystems) {
      stateMachineSubsystem.beforePeriodic();
    }
  }

  public void register(StateMachineSubsystem<?> stateMachine) {
    stateMachineSubsystems.add(stateMachine);
  }
}
