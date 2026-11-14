package com.team581.trailblazer3.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class VectorLimiter {
  public record Result(Translation2d velocity, boolean infeasible) {}

  private record Disk(Translation2d center, double radius) {
    Translation2d project(Translation2d desired) {
      var delta = desired.minus(center);
      double norm = delta.getNorm();
      return norm < EPSILON
          ? center.plus(new Translation2d(radius, 0))
          : center.plus(delta.times(radius / norm));
    }
  }

  private static final class Search {
    final Translation2d desired;
    final Disk[] disks;
    final Translation2d normal;
    final double floor;
    Translation2d best;
    double distance = Double.POSITIVE_INFINITY;

    Search(Translation2d desired, Disk[] disks, Translation2d normal, double floor) {
      this.desired = desired;
      this.disks = disks;
      this.normal = normal;
      this.floor = floor;
    }

    void consider(Translation2d candidate) {
      if (candidate.dot(normal) < floor - EPSILON) {
        return;
      }
      for (Disk disk : disks) {
        if (disk.center().getDistance(candidate) > disk.radius() + EPSILON) {
          return;
        }
      }
      double error = candidate.getDistance(desired);
      if (error < distance) {
        best = candidate;
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
    double speed = previous.getNorm();
    var normal = speed > 1e-6 ? previous.div(speed) : Translation2d.kZero;
    double floor = speed > 1e-6 ? speed - limits.braking() * dt : 0;
    var disks = new Disk[6];
    disks[0] =
        new Disk(Translation2d.kZero, Math.min(limits.maxSpeed(), speed + motorAcceleration * dt));
    disks[1] = new Disk(previous, traction * dt);
    for (int i = 0; i < 4; i++) {
      var location = limits.robot().modulePositionsMeters().get(i).rotateBy(heading);
      disks[i + 2] =
          new Disk(
              new Translation2d(location.getY(), -location.getX()).times(omega),
              limits.maxWheelSpeed());
    }
    var search = new Search(desired, disks, normal, floor);
    search.consider(desired);
    if (search.best != null) {
      return new Result(desired, false);
    }
    if (speed > 1e-6) {
      search.consider(desired.plus(normal.times(floor - desired.dot(normal))));
    }
    for (int i = 0; i < disks.length; i++) {
      var disk = disks[i];
      search.consider(disk.project(desired));
      if (disk.radius() < EPSILON) {
        search.consider(disk.center());
      }
      if (speed > 1e-6) {
        double offset = floor - disk.center().dot(normal);
        if (Math.abs(offset) <= disk.radius()) {
          var midpoint = disk.center().plus(normal.times(offset));
          var span =
              new Translation2d(-normal.getY(), normal.getX())
                  .times(Math.sqrt(Math.max(0, disk.radius() * disk.radius() - offset * offset)));
          search.consider(midpoint.plus(span));
          search.consider(midpoint.minus(span));
        }
      }
      for (int j = 0; j < i; j++) {
        intersect(disk, disks[j], search);
      }
    }
    return search.best != null
        ? new Result(search.best, false)
        : new Result(
            previous.minus(
                normal.times(Math.min(speed, Math.min(traction, limits.braking()) * dt))),
            true);
  }

  private static void intersect(Disk a, Disk b, Search search) {
    var delta = b.center().minus(a.center());
    double distance = delta.getNorm();
    if (distance < EPSILON
        || distance > a.radius() + b.radius()
        || distance < Math.abs(a.radius() - b.radius())) {
      return;
    }
    double along =
        (distance * distance + a.radius() * a.radius() - b.radius() * b.radius()) / (2 * distance);
    var unit = delta.div(distance);
    var midpoint = a.center().plus(unit.times(along));
    var span =
        new Translation2d(-unit.getY(), unit.getX())
            .times(Math.sqrt(Math.max(0, a.radius() * a.radius() - along * along)));
    search.consider(midpoint.plus(span));
    search.consider(midpoint.minus(span));
  }

  private VectorLimiter() {}
}
