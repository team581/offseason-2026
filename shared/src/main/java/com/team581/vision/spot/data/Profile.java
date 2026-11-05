package com.team581.vision.spot.data;

import java.util.List;

public record Profile(
    List<CameraPolicy> cameras,
    FieldLayout field,
    boolean fusionEnabled,
    boolean headingEnabled,
    double maxAgeS,
    double futureToleranceS,
    double reorderWindowS) {
  public Profile {
    cameras = List.copyOf(cameras);
  }
}
