package com.team581.vision.spot.data;

import java.util.List;

public record CameraDefinition(
    String name,
    boolean required,
    boolean enabled,
    Pose robotToCamera,
    int imageRotationDeg,
    List<OpticalMode> allowedModes,
    List<ControlPolicy> controls) {
  public CameraDefinition {
    allowedModes = List.copyOf(allowedModes);
    controls = List.copyOf(controls);
  }
}
