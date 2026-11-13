package frc.robot.testing;

import com.team581.trailblazer.Trailblazer;
import frc.robot.robot_manager.RobotManager;

/** Register tests here; both the dashboard chooser and headless runner use this list. */
public enum TestSelection {
  NONE,
  STRAIGHT_LINE;

  public RobotTest<?> create(RobotManager robotManager, Trailblazer trailblazer) {
    return switch (this) {
      case STRAIGHT_LINE -> new StraightLineTest(robotManager, trailblazer);
      case NONE -> throw new IllegalStateException("NONE has no test to create");
    };
  }
}
