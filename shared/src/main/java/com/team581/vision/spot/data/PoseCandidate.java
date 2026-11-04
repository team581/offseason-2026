package com.team581.vision.spot.data;

import java.util.List;

public record PoseCandidate(
    Pose fieldToCamera,
    PoseMethod method,
    List<Integer> tagIds,
    double reprojectionErrorPx,
    double averageTagDistanceM,
    double imageAreaFraction,
    double imageSpreadFraction,
    double distortionDisplacementPx) {
  public PoseCandidate {
    tagIds = List.copyOf(tagIds);
  }
}
