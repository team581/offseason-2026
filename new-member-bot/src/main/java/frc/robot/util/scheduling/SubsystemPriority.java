package frc.robot.util.scheduling;

import com.team581.util.scheduling.SubsystemPriorityBase;

public enum SubsystemPriority implements SubsystemPriorityBase {
  INTAKE(0);

  private final int value;

  SubsystemPriority(int value) {
    this.value = value;
  }

  @Override
  public int getValue() {
    return value;
  }
}
