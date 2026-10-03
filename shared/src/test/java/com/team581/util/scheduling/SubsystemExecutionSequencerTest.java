package com.team581.util.scheduling;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

final class SubsystemExecutionSequencerTest {
  private static void register(List<String> order, String name, int priority) {
    SubsystemExecutionSequencer.registerSubsystem(
        new Subsystem() {
          @Override
          public SubsystemPriorityBase getPriority() {
            return () -> priority;
          }

          @Override
          public void periodic() {
            order.add(name);
          }
        });
  }

  @Test
  void executesEveryPriorityInOrderWithStableTies() {
    var order = new ArrayList<String>();
    register(order, "actuator", 0);
    register(order, "manager", 29);
    register(order, "vision", 10);
    register(order, "localization", 11);
    register(order, "autos", 30);
    register(order, "inputs", 999);
    register(order, "second actuator", 0);
    SubsystemExecutionSequencer.periodic();
    assertThat(order)
        .containsExactly(
            "inputs", "autos", "manager", "localization", "vision", "actuator", "second actuator");
  }
}
