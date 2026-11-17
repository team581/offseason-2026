package com.team581.trailblazer3.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class HeadingPlanner {
  public record Result(State reference, double command, double acceleration) {}

  public record State(double angle, double velocity) {}

  /**
   * Steps heading tracking using {@code translationSpeed} for kinematic {@code targetOmega} and
   * {@code budgetSpeed} for the rotation/translation friction split.
   */
  public static Result step(
      State reference,
      double target,
      double derivative,
      double measuredAngle,
      double previousOmega,
      double translationSpeed,
      double budgetSpeed,
      DrivetrainLimits limits,
      double dt) {
    double omegaLimit = limits.rotationSpeed(budgetSpeed);
    double alphaLimit = limits.rotationAccel(budgetSpeed);
    double targetOmega = MathUtil.clamp(derivative * translationSpeed, -omegaLimit, omegaLimit);
    double error = reference.angle() - target;
    double rate = reference.velocity() - targetOmega;
    double relativeLimit =
        error < 0 || (error == 0 && rate > 0) ? omegaLimit - targetOmega : omegaLimit + targetOmega;
    double nextRate = rate;
    if (alphaLimit > 1e-6) {
      if (relativeLimit <= 1e-4) {
        nextRate += MathUtil.clamp(-rate, -alphaLimit * dt, alphaLimit * dt);
      } else {
        var trapezoid =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(relativeLimit, alphaLimit));
        nextRate =
            trapezoid.calculate(
                    dt, new TrapezoidProfile.State(error, rate), new TrapezoidProfile.State())
                .velocity;
      }
    }
    double desired = MathUtil.clamp(targetOmega + nextRate, -omegaLimit, omegaLimit);
    double nextOmega =
        reference.velocity()
            + MathUtil.clamp(desired - reference.velocity(), -alphaLimit * dt, alphaLimit * dt);
    double nextAngle = reference.angle() + (reference.velocity() + nextOmega) * dt / 2;
    double gain = Math.min(8, 1 / (3 * (limits.lag() + dt / 2)));
    double demand =
        MathUtil.clamp(
            nextOmega + gain * MathUtil.angleModulus(nextAngle - measuredAngle),
            -omegaLimit,
            omegaLimit);
    double command =
        previousOmega
            + MathUtil.clamp(
                demand - previousOmega, -limits.maxAlpha() * dt, limits.maxAlpha() * dt);
    return new Result(
        new State(nextAngle, nextOmega), command, (nextOmega - reference.velocity()) / dt);
  }

  /**
   * Steps heading tracking using {@code translationSpeed} for both tangent rate and friction share.
   */
  public static Result step(
      State reference,
      double target,
      double derivative,
      double measuredAngle,
      double previousOmega,
      double translationSpeed,
      DrivetrainLimits limits,
      double dt) {
    return step(
        reference,
        target,
        derivative,
        measuredAngle,
        previousOmega,
        translationSpeed,
        translationSpeed,
        limits,
        dt);
  }

  /** Tracks a heading reference at measured path progress, with drive response compensation. */
  public static Result track(
      HeadingSchedule schedule,
      double progress,
      double measuredSpeed,
      double previousSpeed,
      Pose2d measuredPose,
      double measuredOmega,
      double previousOmega,
      DrivetrainLimits limits,
      double dt) {
    double speed = Math.max(0, measuredSpeed);
    var yaw = schedule.sample(progress);
    double rate = yaw.slope() * speed;
    double acceleration =
        MathUtil.clamp(
            (previousSpeed - speed) / Math.max(dt, limits.lag()),
            -limits.braking(),
            limits.acceleration(speed));
    // Look ahead in the feedforward only. The feedback compares poses at the same progress.
    double lead = 2 * limits.lag() + dt / 2;
    double futureSpeed = Math.max(0, speed + lead * acceleration);
    var future = schedule.sample(progress + lead * (speed + futureSpeed) / 2);
    double demand =
        future.slope() * futureSpeed
            + 8 * MathUtil.angleModulus(yaw.angle() - measuredPose.getRotation().getRadians())
            + 0.4 * (rate - measuredOmega);
    double command =
        MathUtil.clamp(
            MathUtil.clamp(demand, -limits.maxOmega(), limits.maxOmega()),
            previousOmega - limits.maxAlpha() * dt,
            previousOmega + limits.maxAlpha() * dt);
    return new Result(new State(yaw.angle(), rate), command, (command - previousOmega) / dt);
  }

  private HeadingPlanner() {}
}
