package frc.robot.testing;

import java.util.Optional;
import java.util.function.Function;

/** Register tests here; both the dashboard chooser and headless runner use these factories. */
public enum TestSelection {
  NONE(manager -> Optional.empty()),
  STRAIGHT_LINE(manager -> Optional.of(new StraightLineRoutine(manager)));

  public final Function<TestManager, Optional<TestManager.Test<?>>> test;

  TestSelection(Function<TestManager, Optional<TestManager.Test<?>>> test) {
    this.test = test;
  }
}
