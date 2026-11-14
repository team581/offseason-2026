package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.SpeedZone;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import edu.wpi.first.math.MathUtil;
import java.util.List;
import java.util.TreeSet;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

public final class VelocityProfiler {
  public record Result(
      double speed,
      double tangentProgress,
      double tangentialAcceleration,
      boolean infeasible,
      LimitingFactor limitingFactor) {}

  public static Result profile(
      PathBuilder.Path path,
      List<Goal> goals,
      List<SpeedZone> zones,
      double measured,
      double startSpeed,
      double barrier,
      DoubleFunction<RotationBudgetEstimator.Budget> budgets,
      DrivetrainLimits limits,
      DoubleUnaryOperator motor,
      double dt) {
    if (path.length() <= 1e-9 || measured >= barrier - 1e-9) {
      return new Result(0, barrier, 0, startSpeed > 1e-6, LimitingFactor.GOAL_DECELERATION);
    }
    if (path.length() > 100) {
      throw new IllegalArgumentException("Path exceeds profile capacity");
    }
    var nodes = new TreeSet<Double>();
    int intervals = (int) Math.ceil(path.length() / 0.05);
    for (int i = 0; i <= intervals; i++) {
      double s = path.length() * i / intervals;
      if (s > measured && s < barrier) {
        nodes.add(s);
      }
    }
    nodes.add(measured);
    nodes.add(barrier);
    for (var piece : path.pieces()) {
      if (piece.end() > measured && piece.end() < barrier) {
        nodes.add(piece.end());
      }
    }
    for (double anchor : path.anchors()) {
      if (anchor > measured && anchor < barrier) {
        nodes.add(anchor);
      }
    }
    for (var zone : zones) {
      if (zone.startMeters() > measured && zone.startMeters() < barrier) {
        nodes.add(zone.startMeters());
      }
      if (zone.endMeters() > measured && zone.endMeters() < barrier) {
        nodes.add(zone.endMeters());
      }
    }
    double[] s = nodes.stream().mapToDouble(Double::doubleValue).toArray();
    double[] u = new double[s.length];
    var factors = new LimitingFactor[s.length];
    double[] curvature = new double[s.length - 1];
    double[] traction = new double[s.length];
    for (int i = 0; i < s.length; i++) {
      var budget = budgets.apply(s[i]);
      u[i] = budget.speed() * budget.speed();
      traction[i] = budget.traction();
      factors[i] = LimitingFactor.TOP_SPEED;
      for (var zone : zones) {
        if (s[i] >= zone.startMeters()
            && s[i] <= zone.endMeters()
            && zone.maxSpeed() * zone.maxSpeed() < u[i]) {
          u[i] = zone.maxSpeed() * zone.maxSpeed();
          factors[i] = LimitingFactor.ZONE_SPEED_CAP;
        }
      }
    }
    for (int i = 0; i < curvature.length; i++) {
      double k = Math.abs(path.sample((s[i] + s[i + 1]) / 2).curvature());
      curvature[i] = k;
      if (k > 0) {
        double turn = limits.turnRate() / k;
        double cap =
            Math.min(
                Math.min(Math.min(traction[i], traction[i + 1]), limits.lateral()) / k,
                turn * turn);
        if (cap < u[i]) {
          u[i] = cap;
          factors[i] = LimitingFactor.CORNER_CENTRIPETAL;
        }
        if (cap < u[i + 1]) {
          u[i + 1] = cap;
          factors[i + 1] = LimitingFactor.CORNER_CENTRIPETAL;
        }
      }
    }
    for (int i = 0; i < goals.size(); i++) {
      int node = java.util.Arrays.binarySearch(s, path.anchors().get(i));
      if (node >= 0) {
        double cap = goals.get(i).passSpeed();
        u[node] = Math.min(u[node], cap * cap);
      }
    }
    u[u.length - 1] = 0;
    factors[u.length - 1] =
        barrier < path.length()
            ? LimitingFactor.HEADING_HARD_GATE
            : LimitingFactor.GOAL_DECELERATION;
    for (int i = u.length - 2; i >= 0; i--) {
      double ds = s[i + 1] - s[i];
      double backward =
          Math.min(
              u[i + 1] + 2 * ds * limits.braking(),
              frictionRoot(u[i + 1], ds, curvature[i], Math.min(traction[i], traction[i + 1])));
      if (backward < u[i]) {
        u[i] = backward;
        factors[i] = factors[i + 1];
      }
    }
    boolean infeasible = startSpeed < -1e-6 || startSpeed * startSpeed > u[0] + 1e-6;
    u[0] = Math.min(u[0], Math.max(0, startSpeed) * Math.max(0, startSpeed));
    for (int i = 0; i < u.length - 1; i++) {
      double ds = s[i + 1] - s[i];
      u[i + 1] =
          Math.min(
              u[i + 1],
              frictionRoot(u[i], ds, curvature[i], Math.min(traction[i], traction[i + 1])));
      if (u[i + 1] <= u[i]) {
        continue;
      }
      double lower = u[i];
      double upper = Math.min(u[i + 1], lower + 2 * ds * motor.applyAsDouble(Math.sqrt(lower)));
      if (upper - lower > 2 * ds * motor.applyAsDouble(Math.sqrt(upper))) {
        for (int iteration = 0; iteration < 4; iteration++) {
          double midpoint = (lower + upper) / 2;
          if (midpoint - u[i] <= 2 * ds * motor.applyAsDouble(Math.sqrt(midpoint))) {
            lower = midpoint;
          } else {
            upper = midpoint;
          }
        }
        upper = lower;
      }
      if (upper + 1e-9 < u[i + 1]) {
        factors[i + 1] = LimitingFactor.MOTOR_ACCELERATION;
      }
      u[i + 1] = upper;
    }
    double speed = Math.sqrt(u[0]);
    double deceleration = Math.min(limits.braking(), traction[0]);
    if (deceleration <= 1e-6) {
      return new Result(0, measured, 0, true, LimitingFactor.INFEASIBLE);
    }
    double lag = dt + limits.lag();
    double taper = Math.max(0.30, u[0] / (2 * deceleration));
    double lead = lag * Math.min(1, (barrier - measured) / taper);
    double lateral = u[0] * curvature[0];
    double available =
        Math.min(
            motor.applyAsDouble(speed),
            Math.sqrt(Math.max(0, traction[0] * traction[0] - lateral * lateral)));
    double evaluation = Math.min(barrier, measured + speed * lead + 0.5 * available * lead * lead);
    int interval = 0;
    while (interval < s.length - 2 && s[interval + 1] < evaluation) {
      interval++;
    }
    double ds = s[interval + 1] - s[interval];
    double delta = u[interval + 1] - u[interval];
    double target =
        ds <= 1e-9
            ? u[interval]
            : MathUtil.interpolate(u[interval], u[interval + 1], (evaluation - s[interval]) / ds);
    return new Result(
        Math.sqrt(target),
        Math.min(barrier, measured + 0.5 * speed * lag),
        ds <= 1e-9 ? 0 : delta / (2 * ds),
        infeasible,
        infeasible ? LimitingFactor.INFEASIBLE : factors[interval + 1]);
  }

  private static double frictionRoot(double u, double ds, double curvature, double traction) {
    double stretch = 1 + 4 * ds * ds * curvature * curvature;
    return (u
            + 2
                * ds
                * Math.sqrt(
                    Math.max(0, stretch * traction * traction - curvature * curvature * u * u)))
        / stretch;
  }

  private VelocityProfiler() {}
}
