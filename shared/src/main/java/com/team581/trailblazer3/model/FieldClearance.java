package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Collections.reverse;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Convex hard keepouts, rectangular robot footprint, and optional clearance margin. */
public final class FieldClearance {
  private final FieldGeometry field;
  private final double halfLength;
  private final double halfWidth;
  private final double cornerRadius;
  private final double margin;
  private final List<List<Translation2d>> keepouts;

  // Precomputed primitive geometry avoids Stream/List/Translation2d heap churn in clearance checks.
  private final double[][] polyX;
  private final double[][] polyY;
  private final double[][] edgeNx;
  private final double[][] edgeNy;
  private final double[][] edgeBoundary;

  public FieldClearance(
      FieldGeometry field,
      double halfLength,
      double halfWidth,
      double margin,
      List<List<Translation2d>> keepouts) {
    this.field = requireNonNull(field);
    requireNonNull(keepouts);
    checkArgument(
        Double.isFinite(halfLength)
            && halfLength > 0
            && Double.isFinite(halfWidth)
            && halfWidth > 0
            && Double.isFinite(margin)
            && margin >= 0,
        "Invalid footprint or margin");
    this.halfLength = halfLength;
    this.halfWidth = halfWidth;
    this.cornerRadius = Math.hypot(halfLength, halfWidth);
    this.margin = margin;
    this.keepouts = keepouts.stream().map(List::copyOf).toList();

    int polygonCount = this.keepouts.size();
    this.polyX = new double[polygonCount][];
    this.polyY = new double[polygonCount][];
    this.edgeNx = new double[polygonCount][];
    this.edgeNy = new double[polygonCount][];
    this.edgeBoundary = new double[polygonCount][];

    for (int k = 0; k < polygonCount; k++) {
      var polygon = this.keepouts.get(k);
      int n = polygon.size();
      checkArgument(
          n >= 3 && !polygon.stream().anyMatch(p -> !Double.isFinite(p.getNorm())),
          "Invalid keepout polygon");
      double[] px = new double[n];
      double[] py = new double[n];
      double[] nx = new double[n];
      double[] ny = new double[n];
      double[] boundary = new double[n];
      for (int i = 0; i < n; i++) {
        var vertex = polygon.get(i);
        px[i] = vertex.getX();
        py[i] = vertex.getY();
      }
      for (int i = 0; i < n; i++) {
        int next = (i + 1) % n;
        double ax = px[i];
        double ay = py[i];
        double edgeX = px[next] - ax;
        double edgeY = py[next] - ay;
        double norm = Math.hypot(edgeX, edgeY);
        checkArgument(norm >= 1e-9, "Duplicate keepout vertices");
        for (int j = 0; j < n; j++) {
          double offX = px[j] - ax;
          double offY = py[j] - ay;
          checkArgument(
              edgeX * offY - edgeY * offX >= -1e-9, "Keepouts must be convex and counterclockwise");
        }
        double unitNx = edgeY / norm;
        double unitNy = -edgeX / norm;
        nx[i] = unitNx;
        ny[i] = unitNy;
        boundary[i] = ax * unitNx + ay * unitNy;
      }
      this.polyX[k] = px;
      this.polyY[k] = py;
      this.edgeNx[k] = nx;
      this.edgeNy[k] = ny;
      this.edgeBoundary[k] = boundary;
    }
  }

  /**
   * Zero-allocation signed clearance for pose {@code (x, y)} with unit heading {@code (cos, sin)}.
   */
  public double clearance(double x, double y, double cos, double sin) {
    double absCos = Math.abs(cos);
    double absSin = Math.abs(sin);
    double extentX = halfLength * absCos + halfWidth * absSin;
    double extentY = halfLength * absSin + halfWidth * absCos;
    double distance =
        Math.min(
            Math.min(x - extentX, field.lengthMeters() - (x + extentX)),
            Math.min(y - extentY, field.widthMeters() - (y + extentY)));

    for (int k = 0; k < polyX.length; k++) {
      double[] px = polyX[k];
      double[] py = polyY[k];
      double minLocalX = Double.POSITIVE_INFINITY;
      double maxLocalX = Double.NEGATIVE_INFINITY;
      double minLocalY = Double.POSITIVE_INFINITY;
      double maxLocalY = Double.NEGATIVE_INFINITY;
      for (int v = 0; v < px.length; v++) {
        double dx = px[v] - x;
        double dy = py[v] - y;
        double localX = dx * cos + dy * sin;
        double localY = -dx * sin + dy * cos;
        if (localX < minLocalX) {
          minLocalX = localX;
        }
        if (localX > maxLocalX) {
          maxLocalX = localX;
        }
        if (localY < minLocalY) {
          minLocalY = localY;
        }
        if (localY > maxLocalY) {
          maxLocalY = localY;
        }
      }
      double sepRobotToPoly =
          Math.max(
              Math.max(minLocalX - halfLength, -maxLocalX - halfLength),
              Math.max(minLocalY - halfWidth, -maxLocalY - halfWidth));

      double[] nx = edgeNx[k];
      double[] ny = edgeNy[k];
      double[] boundary = edgeBoundary[k];
      double sepPolyToRobot = Double.NEGATIVE_INFINITY;
      for (int e = 0; e < nx.length; e++) {
        double enx = nx[e];
        double eny = ny[e];
        double centerProj = x * enx + y * eny;
        double support =
            halfLength * Math.abs(cos * enx + sin * eny)
                + halfWidth * Math.abs(-sin * enx + cos * eny);
        double edgeSep = (centerProj - support) - boundary[e];
        if (edgeSep > sepPolyToRobot) {
          sepPolyToRobot = edgeSep;
        }
      }
      double polyClearance = Math.max(sepRobotToPoly, sepPolyToRobot);
      if (polyClearance < distance) {
        distance = polyClearance;
      }
    }
    return distance;
  }

  public double clearance(Pose2d pose) {
    var rot = pose.getRotation();
    return clearance(pose.getX(), pose.getY(), rot.getCos(), rot.getSin());
  }

  @Override
  public boolean equals(Object obj) {
    return this == obj
        || (obj instanceof FieldClearance other
            && Double.compare(halfLength, other.halfLength) == 0
            && Double.compare(halfWidth, other.halfWidth) == 0
            && Double.compare(margin, other.margin) == 0
            && field.equals(other.field)
            && keepouts.equals(other.keepouts));
  }

  public FieldGeometry field() {
    return field;
  }

  /** Returns a copy of this clearance flipped from the Blue origin to the given alliance. */
  public FieldClearance forAlliance(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return this;
    }
    var flipped = new ArrayList<List<Translation2d>>(keepouts.size());
    for (var polygon : keepouts) {
      var transformed = new ArrayList<Translation2d>(polygon.size());
      for (var vertex : polygon) {
        transformed.add(field.flipPoint(vertex));
      }
      if (field.symmetry() == FieldSymmetry.MIRROR_X) {
        // Mirroring across X flips polygon winding from CCW to CW; reverse to stay convex-CCW.
        reverse(transformed);
      }
      flipped.add(transformed);
    }
    return new FieldClearance(field, halfLength, halfWidth, margin, flipped);
  }

  public double halfLength() {
    return halfLength;
  }

  public double halfWidth() {
    return halfWidth;
  }

  @Override
  public int hashCode() {
    return Objects.hash(field, halfLength, halfWidth, margin, keepouts);
  }

  public List<List<Translation2d>> keepouts() {
    return keepouts;
  }

  public double margin() {
    return margin;
  }

  /**
   * Zero-allocation conservative lower bound on signed clearance at {@code (x, y)} valid for every
   * robot heading angle.
   */
  public double minimumAngleClearance(double x, double y) {
    double distance =
        Math.min(
            Math.min(x - cornerRadius, field.lengthMeters() - (x + cornerRadius)),
            Math.min(y - cornerRadius, field.widthMeters() - (y + cornerRadius)));
    for (int k = 0; k < edgeNx.length; k++) {
      double[] nx = edgeNx[k];
      double[] ny = edgeNy[k];
      double[] boundary = edgeBoundary[k];
      double sepPoly = Double.NEGATIVE_INFINITY;
      for (int e = 0; e < nx.length; e++) {
        double edgeSep = (x * nx[e] + y * ny[e] - cornerRadius) - boundary[e];
        if (edgeSep > sepPoly) {
          sepPoly = edgeSep;
        }
      }
      if (sepPoly < distance) {
        distance = sepPoly;
      }
    }
    return distance;
  }

  @Override
  public String toString() {
    return "FieldClearance[field="
        + field
        + ", halfLength="
        + halfLength
        + ", halfWidth="
        + halfWidth
        + ", margin="
        + margin
        + ", keepouts="
        + keepouts
        + "]";
  }
}
