package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;

/** Drives straight out of the left (depot-side) trench toward the alliance wall. */
final class LeftTrenchAuto extends TrailblazerAuto {
  private static final double TRAVEL_METERS = 2.0;

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-left-trench")
          .withLimits(MotionLimits.robotMaximum())
          .stopAt(CompAutoField.offset(CompAutoField.LEFT_TRENCH_START, -TRAVEL_METERS, 0.0))
          .build();

  LeftTrenchAuto(AutoContext context) {
    super(CompAutoField.LEFT_TRENCH_START, context);
    drive("leave_left_trench", LEAVE_TRENCH_SEGMENT);
  }
}
