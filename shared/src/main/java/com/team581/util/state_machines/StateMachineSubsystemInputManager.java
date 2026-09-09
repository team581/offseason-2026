package com.team581.util.state_machines;

import static java.util.Comparator.comparingInt;

import com.team581.signals.Signals;
import com.team581.util.profiling.LoopTiming;
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
    long aggregateStart = LoopTiming.start();
    long refreshStart = LoopTiming.start();
    Signals.refreshAll();
    LoopTiming.end("Scheduler/Inputs/Signals.refreshAll()", refreshStart);
    for (var stateMachineSubsystem : stateMachineSubsystems) {
      long inputStart = LoopTiming.start();
      stateMachineSubsystem.beforePeriodic();
      LoopTiming.end(stateMachineSubsystem.getInputLoggerName(), inputStart);
    }
    LoopTiming.end("Scheduler/Inputs/Total", aggregateStart);
  }

  public void register(StateMachineSubsystem<?> stateMachine) {
    stateMachineSubsystems.add(stateMachine);
  }
}
