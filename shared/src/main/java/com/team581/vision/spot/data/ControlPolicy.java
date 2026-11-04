package com.team581.vision.spot.data;

public record ControlPolicy(
    String name, double defaultValue, boolean allowOverride, boolean required) {}
