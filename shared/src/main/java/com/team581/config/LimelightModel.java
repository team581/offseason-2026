package com.team581.config;

public enum LimelightModel {
  THREE(640, 480),
  THREEG(1280, 800),
  FOUR(1280, 800);

  public final int width;
  public final int height;

  private LimelightModel(int width, int height) {
    this.width = width;
    this.height = height;
  }
}