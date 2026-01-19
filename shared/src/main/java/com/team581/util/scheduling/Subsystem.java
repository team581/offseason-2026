package com.team581.util.scheduling;

public interface Subsystem {
  SubsystemPriorityBase getPriority();

  void periodic();
}
