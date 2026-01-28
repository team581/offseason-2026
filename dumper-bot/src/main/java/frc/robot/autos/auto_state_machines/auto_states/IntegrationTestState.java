package frc.robot.autos.auto_state_machines.auto_states;

import com.google.common.collect.ImmutableMap;
import java.util.Map;

public enum IntegrationTestState {
  DRIVE_TO_START,
  CLOSE_CENTERED_WITH_HUB,
  BACK_CENTERED_WITH_HUB,
  RIGHT_TRENCH,
  LEFT_RAMP,
  PAUSE;

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> nextState =
      ImmutableMap.ofEntries(
          Map.entry(DRIVE_TO_START, CLOSE_CENTERED_WITH_HUB),
          Map.entry(CLOSE_CENTERED_WITH_HUB, BACK_CENTERED_WITH_HUB),
          Map.entry(BACK_CENTERED_WITH_HUB, RIGHT_TRENCH),
          Map.entry(RIGHT_TRENCH, LEFT_RAMP));

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> previousState =
      ImmutableMap.ofEntries(
          Map.entry(CLOSE_CENTERED_WITH_HUB, DRIVE_TO_START),
          Map.entry(BACK_CENTERED_WITH_HUB, CLOSE_CENTERED_WITH_HUB),
          Map.entry(RIGHT_TRENCH, BACK_CENTERED_WITH_HUB),
          Map.entry(LEFT_RAMP, RIGHT_TRENCH));

  public IntegrationTestState nextState() {
    return nextState.get(this);
  }

  public IntegrationTestState previousState() {
    return previousState.get(this);
  }
}
