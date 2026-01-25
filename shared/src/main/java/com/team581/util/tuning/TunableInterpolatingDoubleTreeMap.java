package com.team581.util.tuning;

import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.DoubleStream;

public class TunableInterpolatingDoubleTreeMap {
  @SafeVarargs
  public static InterpolatingDoubleTreeMap ofEntries(
      String key, Map.Entry<Double, Double>... entries) {
    var map = InterpolatingDoubleTreeMap.ofEntries(entries);

    if (!GlobalConfig.IS_DEVELOPMENT) {
      return map;
    }

    var flatEntries =
        Arrays.stream(entries)
            .flatMapToDouble(entry -> DoubleStream.of(entry.getKey(), entry.getValue()))
            .toArray();

    DogLog.tunable(
        key,
        flatEntries,
        newFlatEntries -> {
          if (newFlatEntries.length % 2 != 0) {
            // Not a complete list of pairs, ignore
            return;
          }

          map.clear();

          for (int i = 0; i < newFlatEntries.length; i += 2) {
            map.put(newFlatEntries[i], newFlatEntries[i + 1]);
          }
        });

    return map;
  }
}
