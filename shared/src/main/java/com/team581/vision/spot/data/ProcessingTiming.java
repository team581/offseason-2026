package com.team581.vision.spot.data;

public record ProcessingTiming(
    double captureToDetectMs, double detectMs, double solveMs, double totalMs) {}
