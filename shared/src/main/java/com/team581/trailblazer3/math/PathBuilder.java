package com.team581.trailblazer3.math;

import static com.google.common.base.Preconditions.checkArgument;

import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
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

    public double curvatureAt(double s) {
      for (int i = 0; i < pieces.size(); i++) {
        var piece = pieces.get(i);
        if (s <= piece.end()) {
          return piece.curvature();
        }
      }
      return pieces.isEmpty() ? 0 : pieces.get(pieces.size() - 1).curvature();
    }

    public Sample sample(double s) {
      for (int i = 0; i < pieces.size(); i++) {
        var piece = pieces.get(i);
        if (s <= piece.end()) {
          return piece.sample(s);
        }
      }
      return pieces.isEmpty()
          ? new Sample(origin, Translation2d.kZero, 0)
          : pieces.get(pieces.size() - 1).sample(length);
    }

    /** Zero-allocation sample filling {@code out = [x, y, tx, ty, curvature]}. */
    public void sampleInto(double s, double[] out) {
      for (int i = 0; i < pieces.size(); i++) {
        var piece = pieces.get(i);
        if (s <= piece.end()) {
          piece.sampleInto(s, out);
          return;
        }
      }
      if (pieces.isEmpty()) {
        out[0] = origin.getX();
        out[1] = origin.getY();
        out[2] = 0;
        out[3] = 0;
        out[4] = 0;
        return;
      }
      pieces.get(pieces.size() - 1).sampleInto(length, out);
    }

    public Projection project(Translation2d position, double lower, double upper) {
      double px = position.getX();
      double py = position.getY();
      double bestProgress = lower;
      double bestDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < pieces.size(); i++) {
        var piece = pieces.get(i);
        double from = Math.max(piece.start(), lower);
        double to = Math.min(piece.end(), upper);
        if (from > to) {
          continue;
        }
        double progress = piece.project(px, py, from, to);
        double distance = piece.distanceToSample(px, py, progress);
        if (distance < bestDistance - EPSILON) {
          bestProgress = progress;
          bestDistance = distance;
        }
      }
      var closest = sample(bestProgress);
      double cx = closest.position().getX();
      double cy = closest.position().getY();
      double tx = closest.tangent().getX();
      double ty = closest.tangent().getY();
      double crossTrack = (px - cx) * -ty + (py - cy) * tx;
      return new Projection(bestProgress, closest, crossTrack, Math.hypot(px - cx, py - cy));
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

    /** Zero-allocation sample filling {@code out = [x, y, tx, ty, curvature]}. */
    public void sampleInto(double s, double[] out) {
      double local = MathUtil.clamp(s - start, 0, length);
      double dx = direction.getX();
      double dy = direction.getY();
      if (curvature == 0) {
        out[0] = entry.getX() + dx * local;
        out[1] = entry.getY() + dy * local;
        out[2] = dx;
        out[3] = dy;
        out[4] = 0;
        return;
      }
      double angle = curvature * local;
      double cos = Math.cos(angle);
      double sin = Math.sin(angle);
      double tx = dx * cos - dy * sin;
      double ty = dx * sin + dy * cos;
      double cx = entry.getX() - dy / curvature;
      double cy = entry.getY() + dx / curvature;
      out[0] = cx + ty / curvature;
      out[1] = cy - tx / curvature;
      out[2] = tx;
      out[3] = ty;
      out[4] = curvature;
    }

    public double project(Translation2d point, double lower, double upper) {
      return project(point.getX(), point.getY(), lower, upper);
    }

    /** Zero-allocation point projection onto this piece within {@code [lower, upper]}. */
    public double project(double px, double py, double lower, double upper) {
      double dx = direction.getX();
      double dy = direction.getY();
      if (curvature == 0) {
        double localDot = (px - entry.getX()) * dx + (py - entry.getY()) * dy;
        return MathUtil.clamp(start + localDot, lower, upper);
      }
      double cx = entry.getX() - dy / curvature;
      double cy = entry.getY() + dx / curvature;
      double rx = px - cx;
      double ry = py - cy;
      double mid = (lower + upper) * 0.5;
      double midLocal = MathUtil.clamp(mid - start, 0, length);
      double midAngle = curvature * midLocal;
      double midCos = Math.cos(midAngle);
      double midSin = Math.sin(midAngle);
      double mtx = dx * midCos - dy * midSin;
      double mty = dx * midSin + dy * midCos;
      double mx = mty / curvature;
      double my = -mtx / curvature;
      double offset = MathUtil.angleModulus(Math.atan2(ry, rx) - Math.atan2(my, mx));
      double candidate = mid + offset / curvature;
      if (Math.hypot(rx, ry) > 1e-6 && candidate >= lower && candidate <= upper) {
        return candidate;
      }
      return distanceToSample(px, py, lower) <= distanceToSample(px, py, upper) ? lower : upper;
    }

    private double distanceToSample(double px, double py, double s) {
      double local = MathUtil.clamp(s - start, 0, length);
      double dx = direction.getX();
      double dy = direction.getY();
      double sx;
      double sy;
      if (curvature == 0) {
        sx = entry.getX() + dx * local;
        sy = entry.getY() + dy * local;
      } else {
        double angle = curvature * local;
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        double tx = dx * cos - dy * sin;
        double ty = dx * sin + dy * cos;
        double cx = entry.getX() - dy / curvature;
        double cy = entry.getY() + dx / curvature;
        sx = cx + ty / curvature;
        sy = cy - tx / curvature;
      }
      return Math.hypot(px - sx, py - sy);
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
    return build(origin, goals, braking, null);
  }

  public static Path build(
      Translation2d origin, List<Goal> goals, double braking, FieldClearance clearance) {
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
      checkArgument(lengths[i + 1] > EPSILON, "Duplicate waypoint at goal-" + i);
      if (lengths[i] <= EPSILON) {
        continue;
      }
      double turn = directions[i + 1].getAngle().minus(directions[i].getAngle()).getRadians();
      double magnitude = Math.abs(turn);
      if (magnitude <= EPSILON) {
        continue;
      }
      checkArgument(
          Math.PI - magnitude > EPSILON && goal.withinMeters() != 0,
          "Corner requires a stop at goal-" + i);
      if (magnitude >= 3 * Math.PI / 4) {
        warnings.add("NEAR_REVERSAL_CORNER goal-" + i);
      }
      double deviation = Math.max(0.5 * goal.withinMeters(), goal.withinMeters() - 0.05);
      double sine = Math.sin(magnitude / 4);
      double radius = deviation * Math.cos(magnitude / 2) / (2 * sine * sine);
      tangents[i] =
          Math.min(radius * Math.tan(magnitude / 2), 0.85 * Math.min(lengths[i], lengths[i + 1]));
      turns[i] = turn;
    }
    if (clearance != null) {
      for (int i = 0; i < count - 1; i++) {
        if (turns[i] == 0) {
          continue;
        }
        var bisectorVec = directions[i + 1].minus(directions[i]);
        double bNorm = bisectorVec.getNorm();
        if (bNorm <= EPSILON) {
          continue;
        }
        var bisector = bisectorVec.div(bNorm);
        var goal = goals.get(i);
        double cosH;
        double sinH;
        if (goal.heading() != null
            && goal.heading().target() instanceof HeadingPlan.Target.Fixed fixed) {
          cosH = fixed.angle().getCos();
          sinH = fixed.angle().getSin();
        } else {
          var avgDir = directions[i].plus(directions[i + 1]);
          double avgNorm = avgDir.getNorm();
          cosH = avgNorm > EPSILON ? avgDir.getX() / avgNorm : directions[i].getX();
          sinH = avgNorm > EPSILON ? avgDir.getY() / avgNorm : directions[i].getY();
        }
        double tanQuarter = Math.tan(Math.abs(turns[i]) / 4.0);
        double tOrig = tangents[i];
        var arcMid = points[i + 1].plus(bisector.times(tOrig * tanQuarter));
        double midClearance = clearance.clearance(arcMid.getX(), arcMid.getY(), cosH, sinH);
        double vertexClearance =
            clearance.clearance(points[i + 1].getX(), points[i + 1].getY(), cosH, sinH);
        if (midClearance < clearance.margin()
            && vertexClearance > midClearance + 1e-3
            && tOrig > 0.02) {
          double lo = 0.02;
          double hi = tOrig;
          for (int iter = 0; iter < 6; iter++) {
            double mid = 0.5 * (lo + hi);
            var probe = points[i + 1].plus(bisector.times(mid * tanQuarter));
            if (clearance.clearance(probe.getX(), probe.getY(), cosH, sinH) >= clearance.margin()) {
              lo = mid;
            } else {
              hi = mid;
            }
          }
          if (lo < tOrig - 1e-4) {
            tangents[i] = lo;
            warnings.add("CORNER_FILLET_CLAMPED goal-" + i);
          }
        }
      }
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
        checkArgument(radius > EPSILON, "Unresolved fillet at goal-" + i);
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
    if (terminal.passType() == Goal.PassType.FINISH_THROUGH) {
      Translation2d finishDir =
          lengths[count - 1] > EPSILON
              ? directions[count - 1]
              : (count >= 2 && lengths[count - 2] > EPSILON
                  ? directions[count - 2]
                  : new Translation2d(
                      terminal.pose().getRotation().getCos(),
                      terminal.pose().getRotation().getSin()));
      checkArgument(
          finishDir.getNorm() > EPSILON && braking > 0,
          "finishThrough requires a direction and braking capacity");
      double desired =
          Math.max(1.5, 1.15 * terminal.passSpeed() * terminal.passSpeed() / (2 * braking));
      double overrun = Math.min(desired, terminal.maxOverrunMeters());
      if (clearance != null) {
        double requiredClearance = Math.max(0.02, clearance.margin());
        HeadingPlan.Target.Fixed fixedHeading =
            terminal.heading() != null
                    && terminal.heading().target() instanceof HeadingPlan.Target.Fixed fixed
                ? fixed
                : null;
        double safe = 0.0;
        int steps = Math.max(1, (int) Math.ceil(overrun / 0.05));
        for (int s = 1; s <= steps; s++) {
          double step = s == steps ? overrun : s * 0.05;
          var probe = entry.plus(finishDir.times(step));
          double dist =
              fixedHeading != null
                  ? clearance.clearance(
                      probe.getX(),
                      probe.getY(),
                      fixedHeading.angle().getCos(),
                      fixedHeading.angle().getSin())
                  : clearance.minimumAngleClearance(probe.getX(), probe.getY());
          if (dist < requiredClearance) {
            break;
          }
          safe = step;
        }
        overrun = safe;
      }
      if (overrun < desired - 1e-6) {
        warnings.add("CRUISE_OVERRUN_LIMITED goal-" + (count - 1));
      }
      pieces.add(new Piece(progress, overrun, entry, finishDir, 0));
      progress += overrun;
    }
    return new Path(origin, pieces, anchors, progress, warnings);
  }

  private static Translation2d left(Translation2d direction) {
    return new Translation2d(-direction.getY(), direction.getX());
  }

  private PathBuilder() {}
}
