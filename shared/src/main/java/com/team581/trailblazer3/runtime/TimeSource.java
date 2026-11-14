package com.team581.trailblazer3.runtime;

@FunctionalInterface
public interface TimeSource {
  double getTimestampSeconds();
}
