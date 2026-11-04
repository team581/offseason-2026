package com.team581.vision.spot.data;

import java.util.List;

public record TagObservation(
    int id,
    List<Double> cornersXyPx,
    int hamming,
    double decisionMargin,
    boolean usedForPose,
    String rejectionReason,
    Double distortionDisplacementPx) {
  public TagObservation {
    cornersXyPx = List.copyOf(cornersXyPx);
  }
}
