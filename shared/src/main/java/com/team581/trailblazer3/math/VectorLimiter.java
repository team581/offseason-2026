package com.team581.trailblazer3.math;

import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class VectorLimiter {
  public record Result(Translation2d velocity, boolean infeasible, LimitingFactor limitingFactor) {}

  private static final class Search {
    final double desiredX;
    final double desiredY;
    final double[] cx;
    final double[] cy;
    final double[] cr;
    final int diskCount;
    final double normalX;
    final double normalY;
    final double floor;
    boolean hasBest;
    double bestX;
    double bestY;
    double distance = Double.POSITIVE_INFINITY;

    Search(
        double desiredX,
        double desiredY,
        double[] cx,
        double[] cy,
        double[] cr,
        int diskCount,
        double normalX,
        double normalY,
        double floor) {
      this.desiredX = desiredX;
      this.desiredY = desiredY;
      this.cx = cx;
      this.cy = cy;
      this.cr = cr;
      this.diskCount = diskCount;
      this.normalX = normalX;
      this.normalY = normalY;
      this.floor = floor;
    }

    void consider(double x, double y) {
      if (x * normalX + y * normalY < floor - EPSILON) {
        return;
      }
      for (int i = 0; i < diskCount; i++) {
        if (Math.hypot(cx[i] - x, cy[i] - y) > cr[i] + EPSILON) {
          return;
        }
      }
      double error = Math.hypot(x - desiredX, y - desiredY);
      if (error < distance) {
        hasBest = true;
        bestX = x;
        bestY = y;
        distance = error;
      }
    }
  }

  private static final double EPSILON = 1e-9;

  public static Result project(
      Translation2d desired,
      Translation2d previous,
      Rotation2d heading,
      double omega,
      double traction,
      double motorAcceleration,
      DrivetrainLimits limits,
      double dt) {
    double prevX = previous.getX();
    double prevY = previous.getY();
    double desiredX = desired.getX();
    double desiredY = desired.getY();
    double speed = Math.hypot(prevX, prevY);
    double normalX = speed > 1e-6 ? prevX / speed : 0.0;
    double normalY = speed > 1e-6 ? prevY / speed : 0.0;
    double floor = speed > 1e-6 ? speed - limits.braking() * dt : 0.0;
    double motorRadius = Math.min(limits.maxSpeed(), speed + motorAcceleration * dt);

    var modules = limits.robot().modulePositionsMeters();
    int moduleCount = modules.size();
    int diskCount = 2 + moduleCount;
    double[] cx = new double[diskCount];
    double[] cy = new double[diskCount];
    double[] cr = new double[diskCount];
    cx[0] = 0.0;
    cy[0] = 0.0;
    cr[0] = motorRadius;
    cx[1] = prevX;
    cy[1] = prevY;
    cr[1] = traction * dt;

    double cosH = heading.getCos();
    double sinH = heading.getSin();
    double maxWheel = limits.maxWheelSpeed();
    for (int i = 0; i < moduleCount; i++) {
      var mod = modules.get(i);
      double lx = mod.getX() * cosH - mod.getY() * sinH;
      double ly = mod.getX() * sinH + mod.getY() * cosH;
      cx[i + 2] = ly * omega;
      cy[i + 2] = -lx * omega;
      cr[i + 2] = maxWheel;
    }

    var search = new Search(desiredX, desiredY, cx, cy, cr, diskCount, normalX, normalY, floor);
    search.consider(desiredX, desiredY);
    if (search.hasBest) {
      return new Result(desired, false, LimitingFactor.NONE);
    }
    if (speed > 1e-6) {
      double shift = floor - (desiredX * normalX + desiredY * normalY);
      search.consider(desiredX + normalX * shift, desiredY + normalY * shift);
    }
    for (int i = 0; i < diskCount; i++) {
      double dx = desiredX - cx[i];
      double dy = desiredY - cy[i];
      double norm = Math.hypot(dx, dy);
      if (norm < EPSILON) {
        search.consider(cx[i] + cr[i], cy[i]);
      } else {
        double scale = cr[i] / norm;
        search.consider(cx[i] + dx * scale, cy[i] + dy * scale);
      }
      if (cr[i] < EPSILON) {
        search.consider(cx[i], cy[i]);
      }
      if (speed > 1e-6) {
        double offset = floor - (cx[i] * normalX + cy[i] * normalY);
        if (Math.abs(offset) <= cr[i]) {
          double mx = cx[i] + normalX * offset;
          double my = cy[i] + normalY * offset;
          double halfChord = Math.sqrt(Math.max(0.0, cr[i] * cr[i] - offset * offset));
          double sx = -normalY * halfChord;
          double sy = normalX * halfChord;
          search.consider(mx + sx, my + sy);
          search.consider(mx - sx, my - sy);
        }
      }
      for (int j = 0; j < i; j++) {
        intersect(cx[i], cy[i], cr[i], cx[j], cy[j], cr[j], search);
      }
    }
    if (search.hasBest) {
      double bx = search.bestX;
      double by = search.bestY;
      return new Result(
          new Translation2d(bx, by),
          false,
          classifyLimiter(
              desiredX, desiredY, bx, by, cx, cy, cr, diskCount, motorRadius, limits.maxSpeed()));
    }
    double stepBack = Math.min(speed, Math.min(traction, limits.braking()) * dt);
    return new Result(
        new Translation2d(prevX - normalX * stepBack, prevY - normalY * stepBack),
        true,
        LimitingFactor.INFEASIBLE);
  }

  private static LimitingFactor classifyLimiter(
      double desiredX,
      double desiredY,
      double candX,
      double candY,
      double[] cx,
      double[] cy,
      double[] cr,
      int diskCount,
      double motorRadius,
      double maxSpeed) {
    if (Math.hypot(desiredX - candX, desiredY - candY) <= 1e-6) {
      return LimitingFactor.NONE;
    }
    for (int i = 2; i < diskCount; i++) {
      if (excess(cx[i], cy[i], cr[i], desiredX, desiredY) > 1e-5
          && Math.abs(excess(cx[i], cy[i], cr[i], candX, candY)) < 1e-4) {
        return LimitingFactor.MODULE_SPEED;
      }
    }
    if (excess(cx[0], cy[0], cr[0], desiredX, desiredY) > 1e-5
        && Math.abs(excess(cx[0], cy[0], cr[0], candX, candY)) < 1e-4) {
      return motorRadius < maxSpeed - 1e-6
          ? LimitingFactor.MOTOR_ACCELERATION
          : LimitingFactor.TOP_SPEED;
    }
    return LimitingFactor.VECTOR_SLEW_LIMIT;
  }

  private static double excess(double cx, double cy, double cr, double px, double py) {
    return Math.hypot(cx - px, cy - py) - cr;
  }

  private static void intersect(
      double ax, double ay, double ar, double bx, double by, double br, Search search) {
    double dx = bx - ax;
    double dy = by - ay;
    double distance = Math.hypot(dx, dy);
    if (distance < EPSILON || distance > ar + br || distance < Math.abs(ar - br)) {
      return;
    }
    double along = (distance * distance + ar * ar - br * br) / (2.0 * distance);
    double ux = dx / distance;
    double uy = dy / distance;
    double mx = ax + ux * along;
    double my = ay + uy * along;
    double halfChord = Math.sqrt(Math.max(0.0, ar * ar - along * along));
    double sx = -uy * halfChord;
    double sy = ux * halfChord;
    search.consider(mx + sx, my + sy);
    search.consider(mx - sx, my - sy);
  }

  private VectorLimiter() {}
}
