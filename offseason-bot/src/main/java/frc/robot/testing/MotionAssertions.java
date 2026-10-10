package frc.robot.testing;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayDeque;
import java.util.Optional;

/** Independent motion checks; does not alter Trailblazer commands or production constraints. */
public final class MotionAssertions {
  private record Sample(double time, double vx, double vy) {}

  private static boolean finite(double time, ChassisSpeeds speeds) {
    return Double.isFinite(time)
        && Double.isFinite(speeds.vxMetersPerSecond)
        && Double.isFinite(speeds.vyMetersPerSecond)
        && Double.isFinite(speeds.omegaRadiansPerSecond);
  }

  private final double velocityLimit;
  private final double accelerationLimit;
  private final ArrayDeque<Sample> measured = new ArrayDeque<>();
  private Optional<Sample> previousCommand = Optional.empty();
  private double peakCommandAcceleration;

  private double peakMeasuredAcceleration;

  public MotionAssertions(double velocityLimit, double accelerationLimit) {
    if (!Double.isFinite(velocityLimit)
        || velocityLimit <= 0
        || !Double.isFinite(accelerationLimit)
        || accelerationLimit <= 0) {
      throw new IllegalArgumentException("Motion limits must be finite and positive");
    }
    this.velocityLimit = velocityLimit;
    this.accelerationLimit = accelerationLimit;
  }

  public Optional<String> check(double time, ChassisSpeeds actual, ChassisSpeeds command) {
    if (!finite(time, actual) || !finite(time, command)) {
      return Optional.of("Nonfinite motion data");
    }
    var previous = previousCommand.orElseThrow();
    if (time <= previous.time()) {
      return Optional.of("Motion timestamps must increase");
    }
    if (time - previous.time() > 0.10 + 1e-6) {
      return Optional.of("Motion sample gap exceeded 100 ms");
    }
    if (Math.hypot(command.vxMetersPerSecond, command.vyMetersPerSecond) > velocityLimit + 0.01) {
      return Optional.of("Commanded velocity exceeded limit");
    }
    if (Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond)
        > velocityLimit + Math.max(0.10, velocityLimit * 0.10)) {
      return Optional.of("Measured velocity exceeded tolerance");
    }
    double commandAcceleration =
        Math.hypot(
                command.vxMetersPerSecond - previous.vx(),
                command.vyMetersPerSecond - previous.vy())
            / (time - previous.time());
    peakCommandAcceleration = Math.max(peakCommandAcceleration, commandAcceleration);
    previousCommand =
        Optional.of(new Sample(time, command.vxMetersPerSecond, command.vyMetersPerSecond));
    // Keep the samples bracketing time - 100 ms, then interpolate that endpoint.
    double target = Math.max(measured.getFirst().time(), time - 0.10);
    measured.addLast(new Sample(time, actual.vxMetersPerSecond, actual.vyMetersPerSecond));
    while (measured.size() > 2) {
      var iterator = measured.iterator();
      iterator.next();
      if (iterator.next().time() > target) {
        break;
      }
      measured.removeFirst();
    }
    var iterator = measured.iterator();
    var left = iterator.next();
    var right = iterator.next();
    double ratio = (target - left.time()) / (right.time() - left.time());
    double vx = left.vx() + ratio * (right.vx() - left.vx());
    double vy = left.vy() + ratio * (right.vy() - left.vy());
    double measuredAcceleration =
        Math.hypot(actual.vxMetersPerSecond - vx, actual.vyMetersPerSecond - vy) / (time - target);
    peakMeasuredAcceleration = Math.max(peakMeasuredAcceleration, measuredAcceleration);
    if (commandAcceleration > accelerationLimit + 0.05) {
      return Optional.of("Commanded vector acceleration exceeded limit");
    }
    if (measuredAcceleration > accelerationLimit + Math.max(0.20, accelerationLimit * 0.20)) {
      return Optional.of("Measured vector acceleration exceeded tolerance");
    }
    return Optional.empty();
  }

  public double peakCommandAcceleration() {
    return peakCommandAcceleration;
  }

  public double peakMeasuredAcceleration() {
    return peakMeasuredAcceleration;
  }

  /** Seed before requesting movement so startup acceleration is included. */
  public void start(double time, ChassisSpeeds initial) {
    if (!finite(time, initial)) {
      throw new IllegalArgumentException("Nonfinite initial motion data");
    }
    previousCommand =
        Optional.of(new Sample(time, initial.vxMetersPerSecond, initial.vyMetersPerSecond));
    measured.clear();
    measured.addLast(new Sample(time, initial.vxMetersPerSecond, initial.vyMetersPerSecond));
    peakCommandAcceleration = 0;
    peakMeasuredAcceleration = 0;
  }
}
