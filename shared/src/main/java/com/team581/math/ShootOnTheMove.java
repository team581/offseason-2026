package com.team581.math;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleUnaryOperator;

public class ShootOnTheMove {
  public record SeparatedVelocityCompensatedGoal(
      double radialVelocity,
      Translation2d radiallyCompensatedGoal,
      double tangentialVelocity,
      Translation2d tangentiallyCompensatedGoal,
      Translation2d fullyCompensatedGoal) {}

  public record ShotSolution(
      Translation2d goal,
      double timeOfFlight,
      double effectiveTimeOfFlight,
      double residualMeters,
      boolean valid) {}

  // Fixed physical prediction horizon; independent of scheduler timing.
  public static final DoubleSubscriber VELOCITY_TIME_CONSTANT_SECONDS =
      DogLog.tunable("ShootOnTheMove/VelocityTimeConstantSeconds", 0.15);

  public static final DoubleSubscriber PREDICTION_HORIZON_SECONDS =
      DogLog.tunable("ShootOnTheMove/PredictionHorizonSeconds", 0.02);
  public static final DoubleSubscriber ANGULAR_TIME_CONSTANT_SECONDS =
      DogLog.tunable("ShootOnTheMove/AngularTimeConstantSeconds", 0.15);
  private static final int MAX_ITERATIONS = 100;
  private static final double SOLUTION_TOLERANCE_METERS = 1e-10;

  // TODO: Do more thorough tuning with integration test, this was quickly tuned
  public static final DoubleSubscriber DRAG_CONSTANT =
      DogLog.tunable("ShootOnTheMoove/DragCoeff", 0.465);

  /** Integral of exp(-drag * t); has units of seconds, and tends to TOF at zero drag. */
  public static double effectiveTimeOfFlight(double tof, double drag) {
    if (!Double.isFinite(tof) || tof < 0.0 || !Double.isFinite(drag) || drag < 0.0) {
      return Double.NaN;
    }
    return drag == 0.0 ? tof : -Math.expm1(-drag * tof) / drag;
  }

  /**
   * First-order response to a held command, with both velocities at the same point and in the same
   * frame. Handles starting, braking and reversal without a sign-dependent branch. Invalid
   * tuning/timing disables prediction rather than producing an unbounded correction.
   */
  public static double predictVelocity(
      double measured, double commanded, double horizonSeconds, double timeConstantSeconds) {
    if (!Double.isFinite(commanded)
        || !Double.isFinite(horizonSeconds)
        || !Double.isFinite(timeConstantSeconds)
        || horizonSeconds <= 0.0
        || timeConstantSeconds <= 0.0) {
      return measured;
    }
    double alpha = -Math.expm1(-horizonSeconds / timeConstantSeconds);
    return measured + (commanded - measured) * alpha;
  }

  private final DoubleUnaryOperator distanceToTimeOfFlight;

  private final String logPrefix;

  public ShootOnTheMove(DoubleUnaryOperator distanceToTimeOfFlight, String name) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
    logPrefix = "ShootOnTheMove/" + name;
  }

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this(distanceToTimeOfFlight, "Default");
  }

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight, String name) {
    this(
        distance -> {
          Double tof = distanceToTimeOfFlight.get(distance);
          return tof == null ? Double.NaN : tof;
        },
        name);
  }

  public double getEffectiveTimeOfFlight(double tof) {
    return effectiveTimeOfFlight(tof, DRAG_CONSTANT.getAsDouble());
  }

  public String getLogPrefix() {
    return logPrefix;
  }

  public double getPredictedVelocity(double measured, double commanded, String name) {
    double horizon = PREDICTION_HORIZON_SECONDS.getAsDouble();
    double tau = VELOCITY_TIME_CONSTANT_SECONDS.getAsDouble();
    double predicted = predictVelocity(measured, commanded, horizon, tau);
    String key = logPrefix + "/Prediction/" + name;
    // These go to the WPILOG in both development and competition modes.
    DogLog.log(key + "/TimestampSeconds", Timer.getFPGATimestamp());
    DogLog.log(key + "/Measured", measured);
    DogLog.log(key + "/Commanded", commanded);
    DogLog.log(key + "/Predicted", predicted);
    DogLog.log(key + "/Correction", predicted - measured);
    DogLog.log(key + "/HorizonSeconds", horizon);
    DogLog.log(key + "/TimeConstantSeconds", tau);
    return predicted;
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds measured, ChassisSpeeds commanded) {
    return separatedGoal(robot, goal, measured, commanded, 0.0);
  }

  public SeparatedVelocityCompensatedGoal getSeparatedVelocityCompensatedGoalWithEffectiveTof(
      Translation2d robot, Translation2d goal, ChassisSpeeds measured, ChassisSpeeds commanded) {
    return separatedGoal(robot, goal, measured, commanded, DRAG_CONSTANT.getAsDouble());
  }

  public Translation2d getVelocityCompensatedGoal(
      Translation2d robot, Translation2d goal, ChassisSpeeds measured, ChassisSpeeds commanded) {
    return getSeparatedVelocityCompensatedGoal(robot, goal, measured, commanded)
        .fullyCompensatedGoal();
  }

  public Translation2d getVelocityCompensatedGoalWithEffectiveTof(
      Translation2d robot, Translation2d goal, ChassisSpeeds measured, ChassisSpeeds commanded) {
    return getSeparatedVelocityCompensatedGoalWithEffectiveTof(robot, goal, measured, commanded)
        .fullyCompensatedGoal();
  }

  /**
   * Solves q = goal - origin - velocity * E(TOF(|q|)). Velocity must be the field-relative velocity
   * of the release point. No prediction or logging is done here. Rejects invalid tables and
   * non-converging solutions instead of silently using iteration five.
   */
  public ShotSolution solve(
      Translation2d origin, Translation2d goal, ChassisSpeeds velocity, double drag) {
    var v = new Translation2d(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
    var compensated = goal;
    double tof = 0.0;
    double effectiveTof = 0.0;
    double residual = Double.POSITIVE_INFINITY;
    for (int i = 0; i < MAX_ITERATIONS; i++) {
      tof = distanceToTimeOfFlight.applyAsDouble(origin.getDistance(compensated));
      if (!Double.isFinite(tof) || tof <= 0.0) {
        break;
      }
      effectiveTof = effectiveTimeOfFlight(tof, drag);
      var next = goal.minus(v.times(effectiveTof));
      residual = next.getDistance(compensated);
      if (!Double.isFinite(residual)) {
        break;
      }
      if (residual <= SOLUTION_TOLERANCE_METERS) {
        return new ShotSolution(compensated, tof, effectiveTof, residual, true);
      }
      compensated = next;
    }
    return new ShotSolution(goal, tof, effectiveTof, residual, false);
  }

  private SeparatedVelocityCompensatedGoal separatedGoal(
      Translation2d robot,
      Translation2d goal,
      ChassisSpeeds measured,
      ChassisSpeeds commanded,
      double drag) {
    double vx = getPredictedVelocity(measured.vxMetersPerSecond, commanded.vxMetersPerSecond, "vx");
    double vy = getPredictedVelocity(measured.vyMetersPerSecond, commanded.vyMetersPerSecond, "vy");
    var solution = solve(robot, goal, new ChassisSpeeds(vx, vy, 0.0), drag);
    var angle = goal.minus(robot).getAngle();
    double radial = vx * angle.getCos() + vy * angle.getSin();
    double tangential = -vx * angle.getSin() + vy * angle.getCos();
    var radialVector = new Translation2d(radial, 0.0).rotateBy(angle);
    var tangentialVector = new Translation2d(0.0, tangential).rotateBy(angle);
    DogLog.log(logPrefix + "/CompensatedGoal", new Pose2d(solution.goal(), Rotation2d.kZero));
    DogLog.log(logPrefix + "/SolutionValid", solution.valid());
    return new SeparatedVelocityCompensatedGoal(
        radial,
        goal.minus(radialVector.times(solution.effectiveTimeOfFlight())),
        tangential,
        goal.minus(tangentialVector.times(solution.effectiveTimeOfFlight())),
        solution.goal());
  }
}
