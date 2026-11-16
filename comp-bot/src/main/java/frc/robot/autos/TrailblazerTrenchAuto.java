package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;

/** Drives straight out of the right trench toward the alliance wall. */
final class TrailblazerTrenchAuto extends TrailblazerAuto {
  private static final double TRAVEL_METERS = 2.0;

  static final Pose2d START = CompAutoField.RIGHT_TRENCH_START;
  static final Pose2d END = CompAutoField.offset(START, -TRAVEL_METERS, 0.0);

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-right-trench")
          .withLimits(MotionLimits.robotMaximum())
          .stopAt(END)
          .build();

  static Pose2d start(Alliance alliance) {
    return CompAutoField.pose(START, alliance);
  }

  TrailblazerTrenchAuto(AutoContext context) {
    super(START, context);
    buildSteps()
        .onStart(() -> {
          robot.homeDeployInAutoRequest();
          robot.homeShooterHoodRequest();
        });
  }

  TrailblazerTrenchAuto(Trailblazer trailblazer, Alliance alliance) {
    super(START, trailblazer, alliance);
    buildSteps();
  }

  private DriveStep buildSteps() {
    return drive("leave_trench", LEAVE_TRENCH_SEGMENT);
  }
}
