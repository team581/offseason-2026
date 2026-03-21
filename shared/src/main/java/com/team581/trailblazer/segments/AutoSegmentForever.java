package com.team581.trailblazer.segments;

import com.team581.trailblazer.AngularConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.LinearConstraintOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import java.util.Optional;

public class AutoSegmentForever extends AutoSegment {
  public AutoSegmentForever(
      List<AutoPoint<?>> points,
      Optional<LinearConstraintOptions> linearConstraints,
      Optional<AngularConstraintOptions> angularConstraints) {
    super(points, linearConstraints, angularConstraints);
  }

  @Override
  public boolean atGoal(Pose2d robotPose, int currentIndex) {
    return false;
  }

  @Override
  public boolean atGoal(Translation2d robotTranslation, int currentIndex) {
    return false;
  }
}
