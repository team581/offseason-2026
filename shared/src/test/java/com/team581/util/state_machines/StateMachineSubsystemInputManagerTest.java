package com.team581.util.state_machines;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

final class StateMachineSubsystemInputManagerTest {
  private static final class Input extends StateMachineSubsystem<State> {
    private final String name;
    private final List<String> order;

    private Input(int priority, String name, List<String> order) {
      super(() -> priority, State.DEFAULT);
      this.name = name;
      this.order = order;
    }

    @Override
    protected void collectInputs() {
      order.add(name);
    }
  }

  private enum State {
    DEFAULT
  }

  @Test
  void gathersMeasurementsBeforeLocalizationBeforeManager() {
    HAL.initialize(500, 0);
    var manager = new StateMachineSubsystemInputManager();
    var order = new ArrayList<String>();
    manager.register(new Input(29, "manager", order));
    manager.register(new Input(11, "localization", order));
    manager.register(new Input(0, "swerve", order));
    manager.register(new Input(10, "vision", order));
    manager.register(new Input(0, "turret", order));
    manager.periodic();
    assertThat(order).containsExactly("swerve", "turret", "vision", "localization", "manager");
  }
}
