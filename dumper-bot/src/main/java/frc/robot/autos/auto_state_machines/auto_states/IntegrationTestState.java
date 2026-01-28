package frc.robot.autos.auto_state_machines.auto_states;

import com.google.common.collect.ImmutableMap;
import java.util.Map;

public enum IntegrationTestState {
  SEGMENT_1_DRIVE_TO_START,
  SEGMENT_2_CLOSE_CENTERED_WITH_HUB,
  SEGMENT_3_BACK_CENTERED_WITH_HUB,
  SEGMENT_4_RIGHT_TRENCH,
  SEGMENT_5_LEFT_RAMP,
  PAUSED;

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> nextState =
      ImmutableMap.ofEntries(
          Map.entry(SEGMENT_1_DRIVE_TO_START, SEGMENT_2_CLOSE_CENTERED_WITH_HUB),
          Map.entry(SEGMENT_2_CLOSE_CENTERED_WITH_HUB, SEGMENT_3_BACK_CENTERED_WITH_HUB),
          Map.entry(SEGMENT_3_BACK_CENTERED_WITH_HUB, SEGMENT_4_RIGHT_TRENCH),
          Map.entry(SEGMENT_4_RIGHT_TRENCH, SEGMENT_5_LEFT_RAMP));

  private static final ImmutableMap<IntegrationTestState, IntegrationTestState> previousState =
      ImmutableMap.ofEntries(
          Map.entry(SEGMENT_2_CLOSE_CENTERED_WITH_HUB, SEGMENT_1_DRIVE_TO_START),
          Map.entry(SEGMENT_3_BACK_CENTERED_WITH_HUB, SEGMENT_2_CLOSE_CENTERED_WITH_HUB),
          Map.entry(SEGMENT_4_RIGHT_TRENCH, SEGMENT_3_BACK_CENTERED_WITH_HUB),
          Map.entry(SEGMENT_5_LEFT_RAMP, SEGMENT_4_RIGHT_TRENCH));

  public IntegrationTestState nextState() {
    return nextState.get(this);
  }

  public IntegrationTestState previousState() {
    return previousState.get(this);
  }
}
