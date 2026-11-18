package com.team581.trailblazer3.math;

import static com.google.common.base.Preconditions.checkArgument;

import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import edu.wpi.first.math.MathUtil;
import java.util.Arrays;
import java.util.List;

public final class VelocityProfiler {
  @FunctionalInterface
  public interface NodeBudgetEvaluator {
    void evaluate(double nodeProgress, double[] outSpeedAndTraction);
  }

  public record Result(
      double speed, double tangentProgress, boolean infeasible, LimitingFactor limitingFactor) {}

  /** Reuses the fixed path grid, its heading limits, and per-tick scratch buffers. */
  public static final class Workspace {
    private static final int HEADING_BUDGET_WIDTH = 4;

    private PathBuilder.Path fixedPath;
    private DrivetrainLimits fixedLimits;
    private List<HeadingSchedule> fixedSchedules = List.of();
    private double[] fixedNodes = new double[0];
    private double[] headingBudgets = new double[0];
    private double[] rawNodes = new double[256];
    private int[] sourceNodes = new int[256];
    private double[] u = new double[256];
    private LimitingFactor[] factors = new LimitingFactor[256];
    private double[] curvature = new double[256];
    private double[] traction = new double[256];
    private double[] yawSlope = new double[256];
    private double[] yawLeft = new double[256];
    private double[] yawRight = new double[256];
    private final double[] budgetScratch = new double[5];
    private final double[] fixedBudgetScratch = new double[5];
    private final double[] geometryScratch = new double[5];

    private void applyHeadingBudgets(
        int node, PathBuilder.Path path, List<HeadingSchedule> schedules, DrivetrainLimits limits) {
      int source = sourceNodes[node];
      if (source < 0) {
        for (var schedule : schedules) {
          schedule.constrain(rawNodes[node], path, limits, budgetScratch, geometryScratch);
        }
        return;
      }
      int offset = source * HEADING_BUDGET_WIDTH;
      if (Double.isNaN(headingBudgets[offset])) {
        fixedBudgetScratch[0] = Double.POSITIVE_INFINITY;
        Arrays.fill(fixedBudgetScratch, 2, 5, 0);
        for (var schedule : schedules) {
          schedule.constrain(rawNodes[node], path, limits, fixedBudgetScratch, geometryScratch);
        }
        headingBudgets[offset] = fixedBudgetScratch[0];
        System.arraycopy(fixedBudgetScratch, 2, headingBudgets, offset + 1, 3);
      }
      if (headingBudgets[offset] != Double.POSITIVE_INFINITY) {
        budgetScratch[0] = Math.min(budgetScratch[0], headingBudgets[offset]);
        System.arraycopy(headingBudgets, offset + 1, budgetScratch, 2, 3);
      }
    }

    private void ensureCapacity(int needed) {
      if (needed <= rawNodes.length) {
        return;
      }
      int nextCapacity = Math.max(needed, rawNodes.length * 2);
      rawNodes = new double[nextCapacity];
      sourceNodes = new int[nextCapacity];
      u = new double[nextCapacity];
      factors = new LimitingFactor[nextCapacity];
      curvature = new double[nextCapacity];
      traction = new double[nextCapacity];
      yawSlope = new double[nextCapacity];
      yawLeft = new double[nextCapacity];
      yawRight = new double[nextCapacity];
    }

    private void prepareFixedNodes(
        PathBuilder.Path path, List<HeadingSchedule> schedules, DrivetrainLimits limits) {
      if (path == fixedPath && limits == fixedLimits && fixedSchedules.equals(schedules)) {
        return;
      }
      int intervals = (int) Math.ceil(path.length() / 0.05);
      int count = intervals + 1 + path.pieces().size() + path.anchors().size();
      for (var schedule : schedules) {
        count += schedule.nodeCount();
      }
      var nodes = new double[count];
      int next = 0;
      for (int i = 0; i <= intervals; i++) {
        nodes[next++] = path.length() * i / intervals;
      }
      for (var piece : path.pieces()) {
        nodes[next++] = piece.end();
      }
      for (double anchor : path.anchors()) {
        nodes[next++] = anchor;
      }
      for (var schedule : schedules) {
        for (int i = 0; i < schedule.nodeCount(); i++) {
          nodes[next++] = schedule.node(i);
        }
      }
      Arrays.sort(nodes);
      ensureCapacity(count + 2);
      fixedPath = path;
      fixedLimits = limits;
      fixedSchedules = List.copyOf(schedules);
      fixedNodes = nodes;
      headingBudgets = new double[schedules.isEmpty() ? 0 : count * HEADING_BUDGET_WIDTH];
      Arrays.fill(headingBudgets, Double.NaN);
    }

    private int selectNodes(double measured, double barrier) {
      // Keep the live endpoints and collapse nearby nodes in the same order as the sorted grid.
      rawNodes[0] = measured;
      sourceNodes[0] = -1;
      int count = 1;
      for (int i = 0; i < fixedNodes.length; i++) {
        double node = fixedNodes[i];
        if (node >= barrier) {
          break;
        }
        if (node - rawNodes[count - 1] > 1e-9) {
          rawNodes[count] = node;
          sourceNodes[count++] = i;
        }
      }
      if (barrier - rawNodes[count - 1] > 1e-9) {
        count++;
      }
      rawNodes[count - 1] = barrier;
      sourceNodes[count - 1] = -1;
      return count;
    }
  }

  public static Result profile(
      PathBuilder.Path path,
      List<Goal> goals,
      double measured,
      double startSpeed,
      double barrier,
      NodeBudgetEvaluator budgets,
      List<HeadingSchedule> headingSchedules,
      DrivetrainLimits limits,
      double dt,
      Workspace workspace) {
    if (path.length() <= 1e-9 || measured >= barrier - 1e-9) {
      boolean overspeed = startSpeed > limits.braking() * dt + 1e-3;
      return new Result(
          0,
          barrier,
          overspeed,
          overspeed ? LimitingFactor.INFEASIBLE : LimitingFactor.GOAL_DECELERATION);
    }
    checkArgument(path.length() <= 100, "Path exceeds profile capacity");
    workspace.prepareFixedNodes(path, headingSchedules, limits);
    int n = workspace.selectNodes(measured, barrier);
    double[] s = workspace.rawNodes;
    if (n < 2) {
      boolean overspeed = startSpeed > limits.braking() * dt + 1e-3;
      return new Result(
          0,
          barrier,
          overspeed,
          overspeed ? LimitingFactor.INFEASIBLE : LimitingFactor.GOAL_DECELERATION);
    }

    double[] u = workspace.u;
    var factors = workspace.factors;
    double[] curvature = workspace.curvature;
    double[] traction = workspace.traction;
    double[] budgetScratch = workspace.budgetScratch;

    for (int i = 0; i < n; i++) {
      Arrays.fill(budgetScratch, 2, 5, 0);
      budgets.evaluate(s[i], budgetScratch);
      if (!headingSchedules.isEmpty()) {
        workspace.applyHeadingBudgets(i, path, headingSchedules, limits);
      }
      workspace.yawSlope[i] = budgetScratch[2];
      workspace.yawLeft[i] = budgetScratch[3];
      workspace.yawRight[i] = budgetScratch[4];
      u[i] = budgetScratch[0] * budgetScratch[0];
      traction[i] = budgetScratch[1];
      factors[i] = LimitingFactor.TOP_SPEED;
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
        double first = yawCornerCap(Math.min(cap, u[i]), k, i, traction[i], workspace, limits);
        double second =
            yawCornerCap(Math.min(cap, u[i + 1]), k, i + 1, traction[i + 1], workspace, limits);
        if (first < u[i]) {
          u[i] = first;
          factors[i] = LimitingFactor.CORNER_CENTRIPETAL;
        }
        if (second < u[i + 1]) {
          u[i + 1] = second;
          factors[i + 1] = LimitingFactor.CORNER_CENTRIPETAL;
        }
      }
    }
    var anchors = path.anchors();
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
      backward = coupledReach(workspace, limits, i, u[i + 1], Math.min(u[i], backward), false);
      if (backward < u[i]) {
        u[i] = backward;
        factors[i] = factors[i + 1];
      }
    }
    double ceilingAtMeasured = u[0];
    double braking = Math.min(limits.braking(), traction[0]);
    double positiveStart = Math.max(0, startSpeed);
    double maxFeasibleStart = Math.sqrt(Math.max(0, u[0])) + braking * dt + 1e-4;
    boolean infeasible = startSpeed < -(braking * dt + 1e-4) || positiveStart > maxFeasibleStart;
    u[0] = Math.min(u[0], positiveStart * positiveStart);
    double speed = Math.sqrt(u[0]);
    double deceleration = Math.min(limits.braking(), traction[0]);
    if (deceleration <= 1e-6) {
      return new Result(0, measured, true, LimitingFactor.INFEASIBLE);
    }
    double lag = dt + limits.lag();
    double taper = Math.max(0.30, u[0] / (2 * deceleration));
    double lead = lag * Math.min(1, (barrier - measured) / taper);
    double lateral = u[0] * curvature[0];
    double available =
        Math.min(
            limits.acceleration(speed),
            Math.sqrt(Math.max(0, traction[0] * traction[0] - lateral * lateral)));
    double evaluation = Math.min(barrier, measured + speed * lead + 0.5 * available * lead * lead);
    int interval = 0;
    while (interval < n - 2 && s[interval + 1] < evaluation) {
      interval++;
    }
    // The full backward pass accounts for downstream braking. The forward pass only needs
    // to propagate acceleration to the two nodes used in the command interpolation.
    for (int i = 0; i <= interval; i++) {
      double ds = s[i + 1] - s[i];
      u[i + 1] =
          Math.min(
              u[i + 1],
              frictionRoot(u[i], ds, curvature[i], Math.min(traction[i], traction[i + 1])));
      u[i + 1] = coupledReach(workspace, limits, i, u[i], u[i + 1], true);
      if (u[i + 1] <= u[i]) {
        continue;
      }
      double lower = u[i];
      double upper = Math.min(u[i + 1], lower + 2 * ds * limits.acceleration(Math.sqrt(lower)));
      if (upper - lower > 2 * ds * limits.acceleration(Math.sqrt(upper))) {
        for (int iteration = 0; iteration < 4; iteration++) {
          double midpoint = (lower + upper) / 2;
          if (midpoint - u[i] <= 2 * ds * limits.acceleration(Math.sqrt(midpoint))) {
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
    double ds = s[interval + 1] - s[interval];
    double target =
        ds <= 1e-9
            ? u[interval]
            : MathUtil.interpolate(u[interval], u[interval + 1], (evaluation - s[interval]) / ds);
    LimitingFactor limitingFactor = factors[interval + 1];
    if (target > ceilingAtMeasured) {
      target = ceilingAtMeasured;
      limitingFactor = factors[0];
    }
    return new Result(
        Math.sqrt(Math.max(0, target)),
        Math.min(barrier, measured + 0.5 * speed * lag),
        infeasible,
        infeasible ? LimitingFactor.INFEASIBLE : limitingFactor);
  }

  private static double coupledReach(
      Workspace w, DrivetrainLimits limits, int i, double from, double cap, boolean accelerating) {
    if (cap <= from
        || (w.yawSlope[i] == 0
            && w.yawSlope[i + 1] == 0
            && w.yawRight[i] == 0
            && w.yawLeft[i + 1] == 0)) {
      return cap;
    }
    double low = from;
    double high = cap;
    double ds = w.rawNodes[i + 1] - w.rawNodes[i];
    for (int k = 0; k < 10; k++) {
      double u = k == 0 ? cap : (low + high) / 2;
      double a = (u - from) / (2 * ds);
      if (!accelerating) {
        a = -a;
      }
      double left = accelerating ? from : u;
      double right = accelerating ? u : from;
      if (yawFits(left, a, w.curvature[i], w.yawSlope[i], w.yawRight[i], w.traction[i], limits)
          && yawFits(
              right,
              a,
              w.curvature[i],
              w.yawSlope[i + 1],
              w.yawLeft[i + 1],
              w.traction[i + 1],
              limits)) {
        if (k == 0) {
          return cap;
        }
        low = u;
      } else {
        high = u;
      }
    }
    return low;
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

  private static double yawCornerCap(
      double cap, double curvature, int i, double traction, Workspace w, DrivetrainLimits limits) {
    if (yawFits(cap, 0, curvature, w.yawSlope[i], w.yawLeft[i], traction, limits)
        && yawFits(cap, 0, curvature, w.yawSlope[i], w.yawRight[i], traction, limits)) {
      return cap;
    }
    double low = 0;
    double high = cap;
    for (int k = 0; k < 10; k++) {
      double mid = (low + high) / 2;
      if (yawFits(mid, 0, curvature, w.yawSlope[i], w.yawLeft[i], traction, limits)
          && yawFits(mid, 0, curvature, w.yawSlope[i], w.yawRight[i], traction, limits)) {
        low = mid;
      } else {
        high = mid;
      }
    }
    return low;
  }

  private static boolean yawFits(
      double u,
      double a,
      double curvature,
      double slope,
      double bend,
      double traction,
      DrivetrainLimits limits) {
    double alpha = bend * u + slope * a;
    double omega = slope * Math.sqrt(u);
    return Math.abs(alpha) <= limits.maxAlpha() + 1e-9
        && Math.hypot(a, curvature * u)
            <= Math.min(traction, limits.availableTraction(omega, alpha)) + 1e-9;
  }

  private VelocityProfiler() {}
}
