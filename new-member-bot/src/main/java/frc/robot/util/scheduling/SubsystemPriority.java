package frc.robot.util.scheduling;

import com.team581.util.scheduling.SubsystemPriorityBase;

public enum SubsystemPriority implements SubsystemPriorityBase {
  INTAKE(0),
  SHOOTER(0),
  FEEDER(0);

  public final int value;

  SubsystemPriority(int priority) {
    this.value = priority;
  }

  @Override
  public int getValue() {
    return value;
  }
}
