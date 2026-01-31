package com.team581.config;

public record CameraConfig(
    LimelightModel model,
    boolean useMt1AndMt2Hybrid,
    double forward,
    double right,
    double up,
    double pitch,
    double yaw,
    double roll) {}
