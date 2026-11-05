package com.team581.vision.spot.data;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.List;

public record CameraDefinition(
    String name,
    boolean required,
    boolean enabled,
    Pose robotToCamera,
    int imageRotationDeg,
    List<OpticalMode> allowedModes,
    List<ControlPolicy> controls,
    String bindingId,
    JsonNode calibration,
    JsonNode timestampQualification,
    DetectorSettings detector) {
  public CameraDefinition {
    allowedModes = List.copyOf(allowedModes);
    controls = List.copyOf(controls);
  }
}
