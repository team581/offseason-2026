package com.team581.trailblazer3.runtime;

import com.team581.trailblazer3.model.TickResult;

@FunctionalInterface
public interface DrivetrainConsumer {
  /** Applies the field-relative trajectory command with zero velocity deadband. */
  void accept(TickResult command);
}
