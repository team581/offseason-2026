package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;

/** An example multi-step auto for student authors. Not registered on the competition chooser. */
final class TwoSegmentAutoExample extends TrailblazerAuto {
  private enum Marker {
    AT_APPROACH
  }

  private static final AutoSegment APPROACH =
      Trailblazer.segment("approach")
          .maxSpeed(2.5)
          .through(CompAutoField.point(1.5, 2.0), g -> g.within(0.35).mark(Marker.AT_APPROACH))
          .stopAt(CompAutoField.pose(2.0, 2.0, 0.0))
          .build();

  private static final AutoSegment CONTINUE =
      Trailblazer.segment("continue").stopAt(CompAutoField.pose(3.0, 2.0, 0.0)).build();

  TwoSegmentAutoExample(AutoContext context) {
    super(CompAutoField.pose(1.0, 2.0, 0.0), context);

    drive("approach", APPROACH).onMarker(Marker.AT_APPROACH, robot::homeDeployInAutoRequest);

    waitFor("wait_for_deploy", 1.5, () -> !robot.hopperManager.deploy.getState().isHoming());

    drive("continue", CONTINUE).everyLoop(robot::intakeAutoRequest).withTimeout(3.0);
  }
}
