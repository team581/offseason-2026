package com.team581.util.scheduling;

public abstract class RegisteredSubsystem implements Subsystem {
  public RegisteredSubsystem() {
    SubsystemExecutionSequencer.registerSubsystem(this);
  }
}
