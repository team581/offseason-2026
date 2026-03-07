package com.team581.math;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.RegionBSPTree2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;

public class ObstructionCalculator {
  private static final Precision.DoubleEquivalence PRECISION =
      Precision.doubleEquivalenceOfEpsilon(1e-6);

  @SafeVarargs
  public static ObstructionCalculator fromTranslations(List<Translation2d>... obstructionCorners) {
    return new ObstructionCalculator(
        Arrays.stream(obstructionCorners)
            .map(corners -> corners.stream().map(ObstructionCalculator::vector2d).toList())
            .toList());
  }

  private static final Vector2D vector2d(Translation2d translation) {
    return Vector2D.of(translation.getX(), translation.getY());
  }

  private final RegionBSPTree2D region;

  /**
   * @param obstructionCorners Corners of polygons representing obstructions on the field.
   */
  public ObstructionCalculator(List<List<Vector2D>> obstructionCorners) {
    region = RegionBSPTree2D.empty();

    for (List<Vector2D> corners : obstructionCorners) {
      ConvexArea area = ConvexArea.convexPolygonFromVertices(corners, PRECISION);
      region.add(area);
    }
  }

  public boolean isObstructed(Translation2d point) {
    return isObstructed(vector2d(point));
  }

  public boolean isObstructed(Translation2d start, Translation2d end) {
    return isObstructed(vector2d(start), vector2d(end));
  }

  public boolean isObstructed(Vector2D point) {
    return region.contains(point);
  }

  public boolean isObstructed(Vector2D start, Vector2D end) {
    return region.contains(start)
        || region.contains(end)
        || region.linecastFirst(Lines.segmentFromPoints(start, end, PRECISION)) != null;
  }
}
