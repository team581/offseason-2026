package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** Convex hard keepouts, rectangular robot footprint, and optional clearance margin. */
public record FieldClearance(
    FieldGeometry field,
    double halfLength,
    double halfWidth,
    double margin,
    List<List<Translation2d>> keepouts) {
  public FieldClearance {
    if (!Double.isFinite(halfLength)
        || halfLength <= 0
        || !Double.isFinite(halfWidth)
        || halfWidth <= 0
        || !Double.isFinite(margin)
        || margin < 0) {
      throw new IllegalArgumentException("Invalid footprint or margin");
    }
    keepouts = keepouts.stream().map(List::copyOf).toList();
    for (var polygon : keepouts) {
      if (polygon.size() < 3 || polygon.stream().anyMatch(p -> !Double.isFinite(p.getNorm()))) {
        throw new IllegalArgumentException("Invalid keepout polygon");
      }
      for (int i = 0; i < polygon.size(); i++) {
        var a = polygon.get(i);
        var b = polygon.get((i + 1) % polygon.size());
        if (a.getDistance(b) < 1e-9) {
          throw new IllegalArgumentException("Duplicate keepout vertices");
        }
        for (var point : polygon) {
          var edge = b.minus(a);
          var offset = point.minus(a);
          if (edge.getX() * offset.getY() - edge.getY() * offset.getX() < -1e-9) {
            throw new IllegalArgumentException("Keepouts must be convex and counterclockwise");
          }
        }
      }
    }
  }

  public double clearance(Pose2d pose) {
    var corners =
        List.of(
            corner(pose, halfLength, halfWidth),
            corner(pose, -halfLength, halfWidth),
            corner(pose, -halfLength, -halfWidth),
            corner(pose, halfLength, -halfWidth));
    double distance = Double.POSITIVE_INFINITY;
    for (var corner : corners) {
      distance = Math.min(distance, Math.min(corner.getX(), field.lengthMeters() - corner.getX()));
      distance = Math.min(distance, Math.min(corner.getY(), field.widthMeters() - corner.getY()));
    }
    for (var keepout : keepouts) {
      // Positive separation on any separating axis proves the polygons are disjoint.
      distance =
          Math.min(distance, Math.max(separation(corners, keepout), separation(keepout, corners)));
    }
    return distance;
  }

  private static Translation2d corner(Pose2d pose, double x, double y) {
    return pose.getTranslation().plus(new Translation2d(x, y).rotateBy(pose.getRotation()));
  }

  private static double separation(List<Translation2d> a, List<Translation2d> b) {
    double maximum = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < a.size(); i++) {
      var edge = a.get((i + 1) % a.size()).minus(a.get(i));
      var normal = new Translation2d(edge.getY(), -edge.getX()).div(edge.getNorm());
      double boundary = a.get(i).dot(normal);
      double nearest = Double.POSITIVE_INFINITY;
      for (var point : b) {
        nearest = Math.min(nearest, point.dot(normal));
      }
      maximum = Math.max(maximum, nearest - boundary);
    }
    return maximum;
  }
}
