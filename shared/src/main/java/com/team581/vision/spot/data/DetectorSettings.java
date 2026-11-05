package com.team581.vision.spot.data;

public record DetectorSettings(String backend, int maxHamming) {
  public static DetectorSettings defaults() {
    return new DetectorSettings("cuda", 2);
  }
}
