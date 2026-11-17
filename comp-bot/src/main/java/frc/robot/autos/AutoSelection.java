package frc.robot.autos;

import com.team581.autos.AutoSelectionBase;
import java.util.Optional;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;

/** Competition and characterization autonomous routines shown on the driver station chooser. */
public enum AutoSelection implements AutoSelectionBase {
  DO_NOTHING("Do Nothing", DoNothingAuto::new),
  RIGHT_TRENCH("Right Trench", RightTrenchAuto::new),
  LEFT_TRENCH("Left Trench", LeftTrenchAuto::new),
  TRAILBLAZER_3_RIGHT_TRENCH("TB3 Right Trench Leave", TrailblazerTrenchAuto::new),
  TRAILBLAZER_3_LEFT_TRENCH("TB3 Left Trench Leave", LeftTrenchLeaveAuto::new),
  TRAILBLAZER_3_TRENCH_ROTATION_TEST(
      "TB3 Trench Rotation Test", TrailblazerTrenchRotationAuto::new),
  TRAILBLAZER_3_CALIBRATE_LAG("TB3 Calibrate Lag"),
  TRAILBLAZER_3_CALIBRATE_TRACTION("TB3 Calibrate Traction"),
  TRAILBLAZER_3_CALIBRATE_BRAKING("TB3 Calibrate Braking");

  private final String displayName;
  private final @Nullable Function<AutoContext, TrailblazerAuto<?>> factory;

  AutoSelection(String displayName) {
    this(displayName, null);
  }

  AutoSelection(String displayName, @Nullable Function<AutoContext, TrailblazerAuto<?>> factory) {
    this.displayName = displayName;
    this.factory = factory;
  }

  @Override
  public String toString() {
    return displayName;
  }

  Optional<TrailblazerAuto<?>> create(AutoContext context) {
    return factory == null ? Optional.empty() : Optional.of(factory.apply(context));
  }
}
