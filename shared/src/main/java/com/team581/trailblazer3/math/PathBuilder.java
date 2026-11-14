package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.Goal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public final class PathBuilder {
  public record Path(
      Translation2d origin,
      List<Piece> pieces,
      List<Double> anchors,
      double length,
      List<String> warnings) {
    public Path {
      pieces = List.copyOf(pieces);
      anchors = List.copyOf(anchors);
      warnings = List.copyOf(warnings);
    }

    public Sample sample(double s) {
      for (var piece : pieces) {
        if (s <= piece.end()) {
          return piece.sample(s);
        }
      }
      return pieces.isEmpty()
          ? new Sample(origin, Translation2d.kZero, 0)
          : pieces.get(pieces.size() - 1).sample(length);
    }

    public Projection project(Translation2d position, double lower, double upper) {
      double bestProgress = lower;
      double bestDistance = Double.POSITIVE_INFINITY;
      for (var piece : pieces) {
        double from = Math.max(piece.start(), lower);
        double to = Math.min(piece.end(), upper);
        if (from > to) {
          continue;
        }
        double progress = piece.project(position, from, to);
        double distance = position.getDistance(piece.sample(progress).position());
        if (distance < bestDistance - EPSILON) {
          bestProgress = progress;
          bestDistance = distance;
        }
      }
      var closest = sample(bestProgress);
      return new Projection(
          bestProgress,
          closest,
          position.minus(closest.position()).dot(closest.normal()),
          position.getDistance(closest.position()));
    }
  }

  public record Piece(
      double start, double length, Translation2d entry, Translation2d direction, double curvature) {
    public double end() {
      return start + length;
    }

    public Sample sample(double s) {
      double local = MathUtil.clamp(s - start, 0, length);
      if (curvature == 0) {
        return new Sample(entry.plus(direction.times(local)), direction, 0);
      }
      double angle = curvature * local;
      var center = entry.plus(left(direction).div(curvature));
      var tangent = direction.rotateBy(Rotation2d.fromRadians(angle));
      return new Sample(center.minus(left(tangent).div(curvature)), tangent, curvature);
    }

    public double project(Translation2d point, double lower, double upper) {
      if (curvature == 0) {
        return MathUtil.clamp(start + point.minus(entry).dot(direction), lower, upper);
      }
      var center = entry.plus(left(direction).div(curvature));
      var radial = point.minus(center);
      var midpoint = sample((lower + upper) / 2).position().minus(center);
      double offset =
          MathUtil.angleModulus(radial.getAngle().minus(midpoint.getAngle()).getRadians());
      double candidate = (lower + upper) / 2 + offset / curvature;
      if (radial.getNorm() > 1e-6 && candidate >= lower && candidate <= upper) {
        return candidate;
      }
      return point.getDistance(sample(lower).position())
              <= point.getDistance(sample(upper).position())
          ? lower
          : upper;
    }
  }

  public record Projection(double progress, Sample sample, double crossTrack, double distance) {}

  public record Sample(Translation2d position, Translation2d tangent, double curvature) {
    public Translation2d normal() {
      return left(tangent);
    }
  }

  private static final double EPSILON = 1e-9;

  public static Path build(Translation2d origin, List<Goal> goals, double braking) {
    int count = goals.size();
    var points = new Translation2d[count + 1];
    var directions = new Translation2d[count];
    var lengths = new double[count];
    var turns = new double[count];
    var tangents = new double[count];
    var scale = new double[count];
    var warnings = new ArrayList<String>();
    points[0] = origin;
    for (int i = 0; i < count; i++) {
      points[i + 1] = goals.get(i).pose().getTranslation();
      var leg = points[i + 1].minus(points[i]);
      lengths[i] = leg.getNorm();
      directions[i] = lengths[i] > EPSILON ? leg.div(lengths[i]) : Translation2d.kZero;
    }
    for (int i = 0; i < count - 1; i++) {
      var goal = goals.get(i);
      if (lengths[i + 1] <= EPSILON) {
        throw new IllegalArgumentException("Duplicate waypoint at " + goal.id());
      }
      if (lengths[i] <= EPSILON) {
        continue;
      }
      double turn = directions[i + 1].getAngle().minus(directions[i].getAngle()).getRadians();
      double magnitude = Math.abs(turn);
      if (magnitude <= EPSILON) {
        continue;
      }
      if (Math.PI - magnitude <= EPSILON || goal.withinMeters() == 0) {
        throw new IllegalArgumentException("Corner requires a stop at " + goal.id());
      }
      if (magnitude >= 3 * Math.PI / 4) {
        warnings.add("NEAR_REVERSAL_CORNER " + goal.id());
      }
      double deviation = Math.max(0.5 * goal.withinMeters(), goal.withinMeters() - 0.05);
      double sine = Math.sin(magnitude / 4);
      double radius = deviation * Math.cos(magnitude / 2) / (2 * sine * sine);
      tangents[i] =
          Math.min(radius * Math.tan(magnitude / 2), 0.85 * Math.min(lengths[i], lengths[i + 1]));
      turns[i] = turn;
    }
    for (int i = 0; i < count; i++) {
      double used = tangents[i] + (i == 0 ? 0 : tangents[i - 1]);
      scale[i] = used > 0.95 * lengths[i] ? 0.95 * lengths[i] / used : 1;
    }
    for (int i = 0; i < count - 1; i++) {
      tangents[i] *= Math.min(scale[i], scale[i + 1]);
    }
    var pieces = new ArrayList<Piece>();
    var anchors = new ArrayList<Double>();
    double progress = 0;
    Translation2d entry = origin;
    for (int i = 0; i < count; i++) {
      double lineLength = lengths[i] - tangents[i] - (i == 0 ? 0 : tangents[i - 1]);
      if (lineLength > EPSILON) {
        pieces.add(new Piece(progress, lineLength, entry, directions[i], 0));
        progress += lineLength;
      }
      entry = points[i + 1];
      if (turns[i] == 0) {
        anchors.add(progress);
      } else {
        double radius = tangents[i] / Math.tan(Math.abs(turns[i]) / 2);
        if (radius <= EPSILON) {
          throw new IllegalArgumentException("Unresolved fillet at " + goals.get(i).id());
        }
        double length = radius * Math.abs(turns[i]);
        pieces.add(
            new Piece(
                progress,
                length,
                entry.minus(directions[i].times(tangents[i])),
                directions[i],
                Math.copySign(1 / radius, turns[i])));
        anchors.add(progress + length / 2);
        progress += length;
        entry = entry.plus(directions[i + 1].times(tangents[i]));
      }
    }
    var terminal = goals.get(count - 1);
    if (terminal.passType() == Goal.PassType.CRUISE_THROUGH) {
      if (lengths[count - 1] <= EPSILON || braking <= 0) {
        throw new IllegalArgumentException(
            "Cruise-through requires a direction and braking capacity");
      }
      double desired =
          Math.max(1.5, 1.15 * terminal.passSpeed() * terminal.passSpeed() / (2 * braking));
      double overrun = Math.min(desired, terminal.maxOverrunMeters());
      if (overrun < desired) {
        warnings.add("CRUISE_OVERRUN_LIMITED " + terminal.id());
      }
      pieces.add(new Piece(progress, overrun, entry, directions[count - 1], 0));
      progress += overrun;
    }
    return new Path(origin, pieces, anchors, progress, warnings);
  }

  private static Translation2d left(Translation2d direction) {
    return new Translation2d(-direction.getY(), direction.getX());
  }

  private PathBuilder() {}
}
