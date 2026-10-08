package com.team581.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import java.util.function.DoubleFunction;
import org.jspecify.annotations.Nullable;

/** Fixed code mount or capture-time joint-history provider. */
public record VisionCamera(
    String name,
    DoubleFunction<Optional<Transform3d>> mountAt,
    VisionProcessor.@Nullable Policy qualityPolicy) {
  public VisionCamera(String name, DoubleFunction<Optional<Transform3d>> mountAt) {
    this(name, mountAt, null);
  }

  public static VisionCamera fixed(String name, Transform3d measuredMount) {
    return new VisionCamera(name, timestamp -> Optional.of(measuredMount));
  }
}
