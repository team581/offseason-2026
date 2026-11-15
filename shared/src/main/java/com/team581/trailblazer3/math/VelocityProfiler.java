package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.SpeedZone;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import edu.wpi.first.math.MathUtil;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

public final class VelocityProfiler {
  @FunctionalInterface
  public interface NodeBudgetEvaluator {
    void evaluate(double nodeProgress, double[] outSpeedAndTraction);
  }

  public record Result(
      double speed,
      double tangentProgress,
      double tangentialAcceleration,
      boolean infeasible,
      LimitingFactor limitingFactor) {}

  /** Reusable scratch buffers avoiding per-tick primitive and enum array allocations. */
  public static final class Workspace {
    private double[] rawNodes = new double[256];
    private double[] u = new double[256];
    private LimitingFactor[] factors = new LimitingFactor[256];
    private double[] curvature = new double[256];
    private double[] traction = new double[256];
    private final double[] budgetScratch = new double[2];

    private void ensureCapacity(int needed) {
      if (needed <= rawNodes.length) {
        return;
      }
      int nextCapacity = Math.max(needed, rawNodes.length * 2);
      rawNodes = new double[nextCapacity];
      u = new double[nextCapacity];
      factors = new LimitingFactor[nextCapacity];
      curvature = new double[nextCapacity];
      traction = new double[nextCapacity];
    }
  }

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
    return profile(
        path,
        goals,
        zones,
        measured,
        startSpeed,
        barrier,
        (node, out) -> {
          var budget = budgets.apply(node);
          out[0] = budget.speed();
          out[1] = budget.traction();
        },
        limits,
        motor,
        dt,
        new Workspace());
  }

  public static Result profile(
      PathBuilder.Path path,
      List<Goal> goals,
      List<SpeedZone> zones,
      double measured,
      double startSpeed,
      double barrier,
      NodeBudgetEvaluator budgets,
      DrivetrainLimits limits,
      DoubleUnaryOperator motor,
      double dt,
      Workspace workspace) {
    if (path.length() <= 1e-9 || measured >= barrier - 1e-9) {
      boolean overspeed = startSpeed > limits.braking() * dt + 1e-3;
      return new Result(
          0,
          barrier,
          0,
          overspeed,
          overspeed ? LimitingFactor.INFEASIBLE : LimitingFactor.GOAL_DECELERATION);
    }
    if (path.length() > 100) {
      throw new IllegalArgumentException("Path exceeds profile capacity");
    }
    int intervals = (int) Math.ceil(path.length() / 0.05);
    var pieces = path.pieces();
    var anchors = path.anchors();
    int zoneCount = zones.size();
    int maxRaw = intervals + 3 + pieces.size() + anchors.size() + 2 * zoneCount;
    workspace.ensureCapacity(maxRaw);

    double[] s = workspace.rawNodes;
    int rawCount = 0;
    for (int i = 0; i <= intervals; i++) {
      double sNode = path.length() * i / intervals;
      if (sNode > measured && sNode < barrier) {
        s[rawCount++] = sNode;
      }
    }
    s[rawCount++] = measured;
    s[rawCount++] = barrier;
    for (int i = 0; i < pieces.size(); i++) {
      double end = pieces.get(i).end();
      if (end > measured && end < barrier) {
        s[rawCount++] = end;
      }
    }
    for (int i = 0; i < anchors.size(); i++) {
      double anchor = anchors.get(i);
      if (anchor > measured && anchor < barrier) {
        s[rawCount++] = anchor;
      }
    }
    for (int i = 0; i < zoneCount; i++) {
      var zone = zones.get(i);
      if (zone.startMeters() > measured && zone.startMeters() < barrier) {
        s[rawCount++] = zone.startMeters();
      }
      if (zone.endMeters() > measured && zone.endMeters() < barrier) {
        s[rawCount++] = zone.endMeters();
      }
    }
    Arrays.sort(s, 0, rawCount);
    int n = 0;
    for (int i = 0; i < rawCount; i++) {
      double value = s[i];
      if (n == 0 || value - s[n - 1] > 1e-9) {
        s[n++] = value;
      } else if (value == barrier) {
        s[n - 1] = barrier;
      }
    }
    if (n < 2) {
      boolean overspeed = startSpeed > limits.braking() * dt + 1e-3;
      return new Result(
          0,
          barrier,
          0,
          overspeed,
          overspeed ? LimitingFactor.INFEASIBLE : LimitingFactor.GOAL_DECELERATION);
    }

    double[] u = workspace.u;
    var factors = workspace.factors;
    double[] curvature = workspace.curvature;
    double[] traction = workspace.traction;
    double[] budgetScratch = workspace.budgetScratch;

    for (int i = 0; i < n; i++) {
      budgets.evaluate(s[i], budgetScratch);
      u[i] = budgetScratch[0] * budgetScratch[0];
      traction[i] = budgetScratch[1];
      factors[i] = LimitingFactor.TOP_SPEED;
      for (int j = 0; j < zoneCount; j++) {
        var zone = zones.get(j);
        if (s[i] >= zone.startMeters()
            && s[i] <= zone.endMeters()
            && zone.maxSpeed() * zone.maxSpeed() < u[i]) {
          u[i] = zone.maxSpeed() * zone.maxSpeed();
          factors[i] = LimitingFactor.ZONE_SPEED_CAP;
        }
      }
    }
    int segmentCount = n - 1;
    for (int i = 0; i < segmentCount; i++) {
      double k = Math.abs(path.curvatureAt((s[i] + s[i + 1]) * 0.5));
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
      int node = Arrays.binarySearch(s, 0, n, anchors.get(i));
      if (node >= 0) {
        double cap = goals.get(i).passSpeed();
        u[node] = Math.min(u[node], cap * cap);
      }
    }
    u[n - 1] = 0;
    factors[n - 1] =
        barrier < path.length()
            ? LimitingFactor.HEADING_HARD_GATE
            : LimitingFactor.GOAL_DECELERATION;
    for (int i = n - 2; i >= 0; i--) {
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
    double braking = Math.min(limits.braking(), traction[0]);
    double positiveStart = Math.max(0, startSpeed);
    double maxFeasibleStart = Math.sqrt(Math.max(0, u[0])) + braking * dt + 1e-4;
    boolean infeasible = startSpeed < -1e-6 || positiveStart > maxFeasibleStart;
    u[0] = Math.min(u[0], positiveStart * positiveStart);
    for (int i = 0; i < n - 1; i++) {
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
    while (interval < n - 2 && s[interval + 1] < evaluation) {
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
