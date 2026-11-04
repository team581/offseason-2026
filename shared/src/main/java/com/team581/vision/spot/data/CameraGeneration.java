package com.team581.vision.spot.data;

public record CameraGeneration(
    String cameraName,
    String bindingId,
    long bindingGeneration,
    String calibrationHash,
    String modeId,
    String opticalSignature,
    boolean calibrationValid) {}
