package frc.robot.util.april_tags;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import org.jspecify.annotations.Nullable;

public class TagMap {
  /**
   * Gets the Pose2d of a specific AprilTag ID.
   *
   * @param id The AprilTag ID
   * @return The Pose2d of the tag (Translation X, Y and Rotation Z), or null if ID is invalid.
   */
  public static @Nullable Pose2d getTagPose(int id) {
    return FieldUtil.FIELD_LAYOUT.getTagPose(id).map(pose3d -> pose3d.toPose2d()).orElse(null);
  }

  private TagMap() {}
}
