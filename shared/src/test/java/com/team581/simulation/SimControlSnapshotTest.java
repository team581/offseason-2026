package com.team581.simulation;

import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.google.common.collect.ImmutableList;
import java.util.Collections;
import org.junit.jupiter.api.Test;

final class SimControlSnapshotTest {
  @Test
  void rejectsInvalidAxes() {
    var invalid =
        new SimControlSnapshot(
            1,
            0,
            false,
            false,
            "teleop",
            "Unknown",
            -1,
            Collections.nCopies(
                6,
                new SimControlSnapshot.Joystick(
                    "x",
                    true,
                    ImmutableList.of(Double.NaN),
                    ImmutableList.of(),
                    ImmutableList.of())));
    assertThatThrownBy(invalid::validate).isInstanceOf(IllegalArgumentException.class);
  }
}
