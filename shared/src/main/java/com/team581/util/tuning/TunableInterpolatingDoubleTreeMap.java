package com.team581.util.tuning;

import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.Map;
import java.util.stream.DoubleStream;

public class TunableInterpolatingDoubleTreeMap {
  @SuppressWarnings("unchecked")
  public static Map<Double, Double> getEntries(InterpolatingDoubleTreeMap map) {
    // Do evil reflection to read the private m_map field from InterpolatingTreeMap
    Field mapField;
    try {
      mapField = InterpolatingTreeMap.class.getDeclaredField("m_map");
    } catch (NoSuchFieldException | SecurityException e) {
      throw new RuntimeException("Failed to get entries from InterpolatingDoubleTreeMap", e);
    }
    mapField.setAccessible(true);

    try {
      return (Map<Double, Double>) mapField.get(map);
    } catch (IllegalArgumentException | IllegalAccessException e) {
      throw new RuntimeException("Failed to get entries from InterpolatingDoubleTreeMap", e);
    }
  }

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
