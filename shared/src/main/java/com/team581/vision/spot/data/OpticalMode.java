package com.team581.vision.spot.data;

public record OpticalMode(
    String id, int width, int height, String pixelFormat, int fps, String opticalSignature) {}
