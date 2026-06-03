package com.team581.math;

import com.team581.util.AprilTags;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.apache.commons.geometry.euclidean.twod.ConvexArea;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.RegionBSPTree2D;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.numbers.core.Precision;
import org.hipparchus.geometry.euclidean.twod.hull.MonotoneChain;
import org.jspecify.annotations.Nullable;

public class ObstructionCalculator {
  private static final Precision.DoubleEquivalence PRECISION =
      Precision.doubleEquivalenceOfEpsilon(1e-6);

  private static final MonotoneChain MONOTONE_CHAIN = new MonotoneChain();

  @SafeVarargs
  public static ObstructionCalculator fromTranslations(
      double expansionMeters, List<Translation2d>... obstructionCorners) {
    return new ObstructionCalculator(
        expansionMeters,
        Arrays.stream(obstructionCorners)
            .map(corners -> corners.stream().map(ObstructionCalculator::vector2d).toList())
            .toList());
  }

  @SafeVarargs
  public static ObstructionCalculator fromTranslations(List<Translation2d>... obstructionCorners) {
    return fromTranslations(0, obstructionCorners);
  }

  /**
   * Expands a convex polygon outward by the given distance. Each edge is offset outward along its
   * normal, and new vertices are computed at the intersections of adjacent offset edges.
   *
   * @param vertices CCW-ordered vertices of the convex polygon.
   * @param amount Distance to expand outward (meters).
   */
  private static List<Vector2D> expandConvexPolygon(List<Vector2D> vertices, double amount) {
    int n = vertices.size();
    if (n < 3) {
      return vertices;
    }

    List<Vector2D> result = new ArrayList<>(n);

    for (int i = 0; i < n; i++) {
      Vector2D prev = vertices.get((i - 1 + n) % n);
      Vector2D curr = vertices.get(i);
      Vector2D next = vertices.get((i + 1) % n);

      // Previous edge (prev -> curr) and current edge (curr -> next)
      Vector2D edge1 = curr.subtract(prev);
      Vector2D edge2 = next.subtract(curr);

      // Offset each edge outward by the expansion amount, then intersect
      Vector2D offsetPoint1 = prev.add(outwardNormal(edge1).multiply(amount));
      Vector2D offsetPoint2 = curr.add(outwardNormal(edge2).multiply(amount));

      Vector2D intersection = lineLineIntersection(offsetPoint1, edge1, offsetPoint2, edge2);
      if (intersection != null) {
        result.add(intersection);
      }
    }

    return result;
  }

  /**
   * Finds the intersection point of two lines, each defined by a point and direction.
   *
   * @return The intersection point, or null if the lines are parallel.
   */
  private static @Nullable Vector2D lineLineIntersection(
      Vector2D p1, Vector2D d1, Vector2D p2, Vector2D d2) {
    double cross = d1.signedArea(d2);
    if (PRECISION.eqZero(Math.abs(cross))) {
      return null;
    }

    double t = p2.subtract(p1).signedArea(d2) / cross;

    return p1.add(d1.multiply(t));
  }

  /**
   * Returns the outward normal for a convex polygon edge (assuming CCW vertex winding).
   *
   * @param edge The edge vector (from one vertex to the next).
   */
  private static Vector2D outwardNormal(Vector2D edge) {
    // orthogonal() returns the CCW perpendicular unit vector, negate for CW (outward normal)
    return edge.orthogonal().negate();
  }

  private static Vector2D vector2d(Translation2d translation) {
    return Vector2D.of(translation.getX(), translation.getY());
  }

  private final RegionBSPTree2D region;

  /**
   * @param expansionMeters Distance to expand each obstruction polygon outward. Use 0 for exact
   *     field geometry.
   * @param obstructionCorners Corners of polygons representing obstructions on the field.
   */
  public ObstructionCalculator(double expansionMeters, List<List<Vector2D>> obstructionCorners) {
    region = RegionBSPTree2D.empty();

    for (List<Vector2D> corners : obstructionCorners) {
      List<org.hipparchus.geometry.euclidean.twod.Vector2D> math3Points =
          corners.stream()
              .map(p -> new org.hipparchus.geometry.euclidean.twod.Vector2D(p.getX(), p.getY()))
              .toList();

      List<Vector2D> hull =
          Arrays.stream(MONOTONE_CHAIN.generate(math3Points).getVertices())
              .map(v -> Vector2D.of(v.getX(), v.getY()))
              .toList();

      if (expansionMeters > 0) {
        hull = expandConvexPolygon(hull, expansionMeters);
      }

      region.add(ConvexArea.convexPolygonFromVertices(hull, PRECISION));
    }
  }

  public boolean contains(Translation2d point) {
    return contains(vector2d(point));
  }

  public boolean contains(Translation2d start, Translation2d end) {
    return contains(vector2d(start), vector2d(end));
  }

  public boolean contains(Vector2D point) {
    return region.contains(point);
  }

  public boolean contains(Vector2D start, Vector2D end) {
    return region.contains(start)
        || region.contains(end)
        || region.linecastFirst(Lines.segmentFromPoints(start, end, PRECISION)) != null;
  }

  public String toSvg() {
    return toSvg(AprilTags.FIELD_LAYOUT.getFieldLength(), AprilTags.FIELD_LAYOUT.getFieldWidth());
  }

  public String toSvg(double fieldLengthMeters, double fieldWidthMeters) {
    double scale = 100.0;
    double svgWidth = fieldLengthMeters * scale;
    double svgHeight = fieldWidthMeters * scale;

    StringBuilder sb = new StringBuilder();
    sb.append(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %s %s\">\n"
            .formatted((int) svgWidth, (int) svgHeight, svgWidth, svgHeight));

    sb.append(
        "<rect width=\"%s\" height=\"%s\" fill=\"#e0e0e0\"/>\n".formatted(svgWidth, svgHeight));

    for (ConvexArea convex : region.toConvex()) {
      List<Vector2D> vertices = convex.getVertices();
      sb.append("<polygon points=\"");
      for (int i = 0; i < vertices.size(); i++) {
        if (i > 0) {
          sb.append(' ');
        }
        // Flip Y so (0,0) is bottom-left
        sb.append(vertices.get(i).getX() * scale);
        sb.append(',');
        sb.append(svgHeight - vertices.get(i).getY() * scale);
      }
      sb.append("\" fill=\"red\"/>\n");
    }

    sb.append("</svg>\n");
    return sb.toString();
  }
}
