package com.team581.vision.spot;

import static java.util.Comparator.comparingInt;

import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.data.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import java.util.List;

/** Geometry and field-layout adapters for Spot. */
public final class SpotProtocol {
  public static Pose3d pose(Pose value) {
    return new Pose3d(
        new Translation3d(value.xM(), value.yM(), value.zM()),
        new Rotation3d(new Quaternion(value.qw(), value.qx(), value.qy(), value.qz())));
  }

  public static Pose pose(Pose3d value) {
    var q = value.getRotation().getQuaternion();
    return new Pose(
        value.getX(), value.getY(), value.getZ(), q.getW(), q.getX(), q.getY(), q.getZ());
  }

  public static Profile profile(AprilTagFieldLayout layout, double tagSizeM) {
    var tags =
        layout.getTags().stream()
            .sorted(comparingInt(t -> t.ID))
            .map(t -> new FieldTag(t.ID, pose(t.pose), tagSizeM))
            .toList();
    return new Profile(
        ImmutableList.of(),
        new FieldLayout("blue-wall-right", layout.getFieldLength(), layout.getFieldWidth(), tags),
        false,
        false,
        .25,
        .005,
        .02);
  }

  public static List<Integer> tagIds(long mask, FieldLayout layout) {
    if (layout.tags().size() > 64
        || (layout.tags().size() < 64 && (mask >>> layout.tags().size()) != 0))
      throw new IllegalArgumentException("Tag mask exceeds field layout");
    var ids = new java.util.ArrayList<Integer>();
    for (int i = 0; i < layout.tags().size(); i++)
      if ((mask & (1L << i)) != 0) ids.add(layout.tags().get(i).id());
    return List.copyOf(ids);
  }

  public static long tagMask(List<Integer> ids, FieldLayout layout) {
    long mask = 0;
    for (int id : ids) {
      int index = -1;
      for (int i = 0; i < layout.tags().size(); i++)
        if (layout.tags().get(i).id() == id) {
          index = i;
          break;
        }
      if (index < 0 || index >= 64)
        throw new IllegalArgumentException("Tag absent from field layout");
      mask |= 1L << index;
    }
    return mask;
  }

  private SpotProtocol() {}
}
