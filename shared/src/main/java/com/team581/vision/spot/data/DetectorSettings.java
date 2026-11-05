package com.team581.vision.spot.data;

public record DetectorSettings(
    String backend,
    String family,
    double decimate,
    double blur,
    int threads,
    boolean refineEdges,
    int maxHamming,
    int minTagPx,
    int minContrast) {
  public static DetectorSettings defaults() {
    return new DetectorSettings("cuda", "tag36h11", 2., 0., 1, true, 2, 16, 3);
  }
}
