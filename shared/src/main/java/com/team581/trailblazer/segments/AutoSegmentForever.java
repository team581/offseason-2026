package com.team581.trailblazer.segments;

import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public class AutoSegmentForever extends AutoSegment {
  public AutoSegmentForever(List<AutoPoint> points, Optional<AutoConstraintOptions> constraints) {
    super(points, constraints);
  }

  @Override
  public boolean atGoal(Pose2d robotPose, int currentIndex) {
    return false;
  }
}
