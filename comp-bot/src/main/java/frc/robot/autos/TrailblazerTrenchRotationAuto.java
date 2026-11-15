package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;

/**
 * Leaves the trench perimeter and rotates 180 degrees counterclockwise while strafing toward the
 * interior of the field.
 */
final class TrailblazerTrenchRotationAuto extends TrailblazerAuto {
  private static final Pose2d START = TrailblazerTrenchAuto.END;
  // Positive blue-field Y is left as seen from the driver station, toward the field interior.
  private static final Pose2d END =
      new Pose2d(START.getX(), START.getY() + 3.0, START.getRotation().plus(Rotation2d.k180deg));

  static final AutoSegment SEGMENT =
      Trailblazer.segment("trench-rotation").stopAt(END, Goal::counterClockwise).build();

  static Pose2d start(Alliance alliance) {
    return CompAutoField.pose(START, alliance);
  }

  TrailblazerTrenchRotationAuto(AutoContext context) {
    super(START, context);
    buildSteps();
  }

  TrailblazerTrenchRotationAuto(Trailblazer trailblazer, Alliance alliance) {
    super(START, trailblazer, alliance);
    buildSteps();
  }

  private void buildSteps() {
    drive("drive_and_turn", SEGMENT);
  }
}
