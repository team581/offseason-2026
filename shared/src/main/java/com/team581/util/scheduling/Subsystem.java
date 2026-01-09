package com.team581.util.scheduling;

public interface Subsystem {
  void periodic();

  SubsystemPriorityBase getPriority();
}
