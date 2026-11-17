package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.HeadingPlan.TurnDirection;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;

/** Continuous heading reference indexed by arc length along the unchanged translation curve. */
public final class HeadingSchedule {
  public record Sample(double angle, double slope, double curvature) {}

  private record Anchor(double progress, double angle, double tolerance, TurnDirection direction) {}

  public static boolean applies(Goal goal) {
    return goal.passType() == Goal.PassType.THROUGH
        && goal.heading() != null
        && goal.heading().explicitTolerance()
        && goal.heading().target() instanceof HeadingPlan.Target.Fixed;
  }

  /** Builds one schedule for each consecutive run of heading tolerance points. */
  public static List<HeadingSchedule> build(
      PathBuilder.Path path,
      List<Goal> goals,
      double initialAngle,
      double turnHold,
      FieldClearance clearance) {
    var sections = new ArrayList<HeadingSchedule>();
    var anchors = new ArrayList<Anchor>();
    double from = turnHold;
    double initial = initialAngle;
    double angle = initial;
    double previous = 0;
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      var frame = goal.heading();
      if (frame == null) {
        continue;
      }
      double anchor =
          MathUtil.clamp(path.anchors().get(i) + frame.offsetMeters(), previous, path.length());
      if (applies(goal) && anchor > Math.max(from, previous) + 1e-6) {
        var fixed = (HeadingPlan.Target.Fixed) frame.target();
        angle += turnDelta(angle, fixed.angle().getRadians(), frame.direction());
        anchors.add(new Anchor(anchor, angle, frame.toleranceRadians(), frame.direction()));
      } else {
        addSection(sections, from, initial, anchors, path, clearance);
        anchors.clear();
        from = Math.max(turnHold, anchor);
        if (frame.target() instanceof HeadingPlan.Target.Fixed fixed) {
          angle += turnDelta(angle, fixed.angle().getRadians(), frame.direction());
        } else if (frame.target() instanceof HeadingPlan.Target.Travel) {
          angle +=
              turnDelta(
                  angle, path.sample(anchor).tangent().getAngle().getRadians(), frame.direction());
        } else if (frame.target() instanceof HeadingPlan.Target.Point point) {
          var delta = point.target().minus(path.sample(anchor).position());
          if (delta.getNorm() >= 0.25) {
            angle +=
                turnDelta(
                    angle,
                    delta.getAngle().getRadians() - point.mechanismOffset().getRadians(),
                    frame.direction());
          }
        }
        initial = angle;
      }
      previous = anchor;
    }
    addSection(sections, from, initial, anchors, path, clearance);
    return List.copyOf(sections);
  }

  public static double turnDelta(double from, double target, TurnDirection direction) {
    double diff = MathUtil.angleModulus(target - from);
    if (direction == TurnDirection.CW && diff > 0) {
      return diff - 2 * Math.PI;
    }
    if (direction == TurnDirection.CCW && diff < 0) {
      return diff + 2 * Math.PI;
    }
    return diff;
  }

  private static void addSection(
      List<HeadingSchedule> sections,
      double from,
      double initial,
      List<Anchor> anchors,
      PathBuilder.Path path,
      FieldClearance clearance) {
    if (anchors.isEmpty()) {
      return;
    }
    var section = new HeadingSchedule(from, initial, anchors, true);
    if (clearance != null && !section.clears(path, clearance)) {
      section = new HeadingSchedule(from, initial, anchors, false);
    }

    if (clearance == null || section.clears(path, clearance)) {
      sections.add(section);
    }
  }

  private final double[] s;

  private final double[] angle;
  private final double[] slope;

  private HeadingSchedule(double start, double initial, List<Anchor> anchors, boolean smooth) {
    s = new double[anchors.size() + 1];
    angle = new double[s.length];
    slope = new double[s.length];
    s[0] = start;
    angle[0] = initial;
    for (int i = 1; i < s.length; i++) {
      var anchor = anchors.get(i - 1);
      s[i] = anchor.progress();
      angle[i] = anchor.angle();
    }
    for (int sweep = 0; smooth && sweep < 12; sweep++) {
      updateSlopes();
      for (int i = s.length - 2; i > 0; i--) {
        var anchor = anchors.get(i - 1);
        var next = anchors.get(i);
        if ((i == 1 && Math.abs(anchor.angle() - initial) < 1e-9)
            || anchor.direction() != TurnDirection.SHORTEST
            || next.direction() != TurnDirection.SHORTEST) {
          continue;
        }
        double left = s[i] - s[i - 1];
        double right = s[i + 1] - s[i];
        double idealFromLeft = angle[i - 1] + (slope[i - 1] + slope[i]) * left / 2;
        double idealFromRight = angle[i + 1] - (slope[i] + slope[i + 1]) * right / 2;
        double weight = 1 / (1 + Math.pow(right / left, 3));

        double margin =
            Math.min(
                0.82 * anchor.tolerance(), Math.max(0, anchor.tolerance() - Math.toRadians(1.7)));
        angle[i] =
            MathUtil.clamp(
                MathUtil.interpolate(idealFromLeft, idealFromRight, weight),
                anchor.angle() - margin,
                anchor.angle() + margin);
      }
    }
    updateSlopes();
  }

  public double end() {
    return s[s.length - 1];
  }

  public Sample sample(double progress) {
    if (progress <= start()) {
      return new Sample(angle[0], 0, 0);
    }
    if (progress >= end()) {
      return new Sample(angle[angle.length - 1], 0, 0);
    }
    int i = 0;
    while (progress > s[i + 1]) {
      i++;
    }
    double width = s[i + 1] - s[i];
    double ds = progress - s[i];
    double average = (angle[i + 1] - angle[i]) / width;
    double b = (3 * average - 2 * slope[i] - slope[i + 1]) / width;
    double c = (-2 * average + slope[i] + slope[i + 1]) / (width * width);
    return new Sample(
        angle[i] + ds * (slope[i] + ds * (b + ds * c)),
        slope[i] + ds * (2 * b + 3 * ds * c),
        2 * b + 6 * ds * c);
  }

  public double start() {
    return s[0];
  }

  private boolean clears(PathBuilder.Path path, FieldClearance clearance) {
    double[] geometry = new double[5];
    for (int i = 0; i < s.length - 1; i++) {
      int steps = Math.max(1, (int) Math.ceil((s[i + 1] - s[i]) / 0.025));
      for (int j = 0; j <= steps; j++) {
        double progress = MathUtil.interpolate(s[i], s[i + 1], (double) j / steps);
        double angle = sample(progress).angle();
        path.sampleInto(progress, geometry);
        if (clearance.clearance(geometry[0], geometry[1], Math.cos(angle), Math.sin(angle)) < 0) {
          return false;
        }
      }
    }
    return true;
  }

  private void updateSlopes() {
    for (int i = 1; i < s.length - 1; i++) {
      double left = (angle[i] - angle[i - 1]) / (s[i] - s[i - 1]);
      double right = (angle[i + 1] - angle[i]) / (s[i + 1] - s[i]);
      slope[i] = left * right > 0 ? 2 * left * right / (left + right) : 0;
    }
  }

  void constrain(double progress, PathBuilder.Path path, DrivetrainLimits limits, double[] budget) {
    if (progress < start() || progress > end()) {
      return;
    }
    var yaw = sample(progress);
    double gradient = Math.abs(yaw.slope());
    double left = sample(progress - 1e-7).curvature();
    double right = sample(progress + 1e-7).curvature();
    double curvature = Math.max(Math.abs(left), Math.abs(right));
    if (gradient < 1e-9 && curvature < 1e-9) {
      return;
    }
    var geometry = path.sample(progress);
    var tangent = geometry.tangent().rotateBy(Rotation2d.fromRadians(-yaw.angle()));
    for (var module : limits.robot().modulePositionsMeters()) {
      double x = tangent.getX() - yaw.slope() * module.getY();
      double y = tangent.getY() + yaw.slope() * module.getX();
      budget[0] = Math.min(budget[0], limits.maxWheelSpeed() / Math.hypot(x, y));
    }
    if (gradient > 1e-9) {
      budget[0] = Math.min(budget[0], limits.maxOmega() / gradient);
    }
    if (curvature > 1e-9) {
      budget[0] = Math.min(budget[0], Math.sqrt(limits.maxAlpha() / curvature));
    }
    budget[2] = yaw.slope();
    budget[3] = left;
    budget[4] = right;
  }

  double node(int index) {
    return s[index];
  }

  int nodeCount() {
    return s.length;
  }
}
