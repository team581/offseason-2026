package com.team581.trailblazer3.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class HeadingPlanner {
  public record Result(State reference, double command, double acceleration) {}

  public record State(double angle, double velocity) {}

  public static Result step(
      State reference,
      double target,
      double derivative,
      double measuredAngle,
      double previousOmega,
      double translationSpeed,
      DrivetrainLimits limits,
      double dt) {
    double omegaLimit = limits.rotationSpeed(translationSpeed);
    double alphaLimit = limits.rotationAccel(translationSpeed);
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

  private HeadingPlanner() {}
}
