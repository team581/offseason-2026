package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import java.util.Optional;
import java.util.function.Function;

/** Competition and characterization autonomous routines shown on the driver station chooser. */
public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING("Do Nothing", DoNothingAuto::new),
  RIGHT_TRENCH("Right Full Auto", RightTrenchAuto::new),
  TRAILBLAZER_3_RIGHT_TRENCH("Right Trench Straight Line", TrailblazerTrenchAuto::new),
  TRAILBLAZER_3_TRENCH_ROTATION_TEST(
      "Post Right Trench Line Rotation Test", TrailblazerTrenchRotationAuto::new);

  private final String displayName;
  private final Optional<Function<AutoContext, TrailblazerAuto<?>>> factory;

  AutoSelection(String displayName) {
    this.displayName = displayName;
    this.factory = Optional.empty();
  }

  AutoSelection(String displayName, Function<AutoContext, TrailblazerAuto<?>> factory) {
    this.displayName = displayName;
    this.factory = Optional.of(factory);
  }

  @Override
  public String toString() {
    return displayName;
  }

  Optional<TrailblazerAuto<?>> create(AutoContext context) {
    return factory.map(create -> create.apply(context));
  }
}
