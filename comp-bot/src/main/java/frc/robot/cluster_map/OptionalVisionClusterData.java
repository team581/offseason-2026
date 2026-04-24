package frc.robot.cluster_map;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import com.team581.util.ReusableOptional;
import edu.wpi.first.math.geometry.Translation2d;

class OptionalVisionClusterData extends ReusableOptional<VisionClusterData> {
  OptionalVisionClusterData() {
    super(new VisionClusterData());
  }

  @CanIgnoreReturnValue
  OptionalVisionClusterData empty() {
    this.isPresent = false;
    return this;
  }

  @CanIgnoreReturnValue
  OptionalVisionClusterData update(Translation2d translation, int size, double score) {
    this.value.update(translation, size, score);
    this.isPresent = true;
    return this;
  }
}
