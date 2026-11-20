package frc.robot.autos;

import com.ctre.phoenix6.Utils;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompDriveConfig;
import java.util.Optional;
import java.util.function.DoubleSupplier;

final class TestTrailblazer {
  static Trailblazer create() {
    return create(Utils::getCurrentTimeSeconds);
  }

  static Trailblazer create(DoubleSupplier clock) {
    return new Trailblazer(
        clock,
        CompDriveConfig.DRIVE_SPEC,
        CompDriveConfig.CALIBRATION,
        CompDriveConfig.FOLLOWER_PERIOD_SECONDS,
        Optional.of(CompAutoField.CLEARANCE));
  }

  private TestTrailblazer() {}
}
