package com.team581.util.tuning;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

final class TunableInterpolatingDoubleTreeMapTest {
  @Test
  void getEntries() {
    var interpolationTable =
        InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(1.0, 1.0), Map.entry(2.0, 2.0), Map.entry(3.0, 3.0));

    var entries = TunableInterpolatingDoubleTreeMap.getEntries(interpolationTable);

    assertThat(entries)
        .containsExactly(Map.entry(1.0, 1.0), Map.entry(2.0, 2.0), Map.entry(3.0, 3.0));
  }
}
