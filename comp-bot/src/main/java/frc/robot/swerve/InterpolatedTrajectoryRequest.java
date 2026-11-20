package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.CompTrailblazer;

/** The publisher owns the ramp timeline. Phoenix samples a single immutable ramp per callback. */
final class InterpolatedTrajectoryRequest implements SwerveRequest {
  private record AngularRamp(double startOmega, double endOmega, AxisTiming timing) {
    private static AngularRamp direct(double omega) {
      return new AngularRamp(omega, omega, AxisTiming.DIRECT);
    }

    private double acceleration(double elapsed) {
      return timing.acceleration(elapsed, startOmega, endOmega);
    }

    private double velocity(double elapsed) {
      return timing.velocity(elapsed, startOmega, endOmega);
    }
  }

  private record AxisTiming(double duration, double grace, double rate) {
    private static final AxisTiming DIRECT = new AxisTiming(0, 0, 0);

    private static double continuationGrace(
        double duration,
        boolean holding,
        double startSpeed,
        double endSpeed,
        double deltaNormSq,
        double deltaDotEnd) {
      if (holding || duration <= EPSILON || deltaNormSq <= EPSILON * EPSILON) {
        return 0;
      }
      if (endSpeed <= STOP_GRACE_SPEED_THRESHOLD && endSpeed <= startSpeed) {
        return 0;
      }
      if (deltaDotEnd < 0) {
        double timeToClosestApproach = (-deltaDotEnd / deltaNormSq) * duration;
        return Math.min(MAX_JITTER_CONTINUATION_SECONDS, timeToClosestApproach);
      }
      return MAX_JITTER_CONTINUATION_SECONDS;
    }

    private static double continuousSlewDuration(
        double stepDuration,
        double startTime,
        boolean holding,
        double previousEnd,
        double previousGrace,
        double distance,
        double rate) {
      double remaining = Math.max(0, previousEnd - startTime);
      if (holding) {
        if (remaining > 0 || distance <= EPSILON) {
          return remaining;
        }
        return rate <= EPSILON ? stepDuration : Math.min(stepDuration, distance / rate);
      }
      double overrun = Math.min(previousGrace, Math.max(0, startTime - previousEnd));
      double baseDuration = Math.max(0, stepDuration - overrun);
      double duration = rate <= EPSILON ? baseDuration : Math.max(baseDuration, distance / rate);
      // Finishing any remaining old slew (or recovering from a grace overrun) plus executing the
      // new step cannot exceed the larger of the two slew rates.
      return Math.min(duration, remaining + overrun + stepDuration);
    }

    private double acceleration(double elapsed, double start, double end) {
      if (duration <= EPSILON || !(elapsed >= 0 && elapsed < duration + grace - EPSILON)) {
        return 0;
      }
      return (end - start) / duration;
    }

    private double velocity(double elapsed, double start, double end) {
      if (duration <= EPSILON || !Double.isFinite(elapsed)) {
        return end;
      }
      if (elapsed < 0) {
        return start;
      }
      double activeTime = Math.min(elapsed, duration + grace);
      return start + ((end - start) / duration) * activeTime;
    }
  }

  private record LinearRamp(
      double startVx, double startVy, double endVx, double endVy, AxisTiming timing) {
    private static LinearRamp direct(double vx, double vy) {
      return new LinearRamp(vx, vy, vx, vy, AxisTiming.DIRECT);
    }

    private double ax(double elapsed) {
      return timing.acceleration(elapsed, startVx, endVx);
    }

    private double ay(double elapsed) {
      return timing.acceleration(elapsed, startVy, endVy);
    }

    private double vx(double elapsed) {
      return timing.velocity(elapsed, startVx, endVx);
    }

    private double vy(double elapsed) {
      return timing.velocity(elapsed, startVy, endVy);
    }
  }

  private record Ramp(VelocityStep step, double startTime, LinearRamp linear, AngularRamp angular) {
    private static Ramp direct(ChassisSpeeds v) {
      return new Ramp(
          new VelocityStep(v, 0),
          0,
          LinearRamp.direct(v.vxMetersPerSecond, v.vyMetersPerSecond),
          AngularRamp.direct(v.omegaRadiansPerSecond));
    }

    private boolean hasStep() {
      return step.durationSeconds() > EPSILON;
    }

    private void sample(double time, ChassisSpeeds v, ChassisSpeeds a) {
      double elapsed = time - startTime;
      v.vxMetersPerSecond = linear.vx(elapsed);
      v.vyMetersPerSecond = linear.vy(elapsed);
      v.omegaRadiansPerSecond = angular.velocity(elapsed);
      a.vxMetersPerSecond = linear.ax(elapsed);
      a.vyMetersPerSecond = linear.ay(elapsed);
      a.omegaRadiansPerSecond = angular.acceleration(elapsed);
    }
  }

  private static final double EPSILON = 1e-9;
  private static final double MAX_JITTER_CONTINUATION_SECONDS = 0.008;
  private static final double STOP_GRACE_SPEED_THRESHOLD = 0.05;

  private static AngularRamp buildAngularRamp(
      Ramp old,
      boolean canContinue,
      VelocityStep step,
      ChassisSpeeds end,
      double startTime,
      double stepDuration) {
    double endOmega = end.omegaRadiansPerSecond;
    double startOmega = step.startOmega();
    double angularRate = Math.abs(endOmega - startOmega) / stepDuration;
    double duration = stepDuration;
    boolean holding = false;

    if (canContinue && MathUtil.isNear(old.angular.endOmega(), startOmega, EPSILON)) {
      startOmega = old.angular.velocity(startTime - old.startTime);
      holding = MathUtil.isNear(old.angular.endOmega(), endOmega, EPSILON);
      if (angularRate <= EPSILON) {
        angularRate = old.angular.timing().rate();
      }
      duration =
          AxisTiming.continuousSlewDuration(
              stepDuration,
              startTime,
              holding,
              old.startTime + old.angular.timing().duration(),
              old.angular.timing().grace(),
              Math.abs(endOmega - startOmega),
              angularRate);
    }

    double dOmega = endOmega - startOmega;
    double grace =
        AxisTiming.continuationGrace(
            duration,
            holding,
            Math.abs(startOmega),
            Math.abs(endOmega),
            dOmega * dOmega,
            dOmega * endOmega);
    return new AngularRamp(startOmega, endOmega, new AxisTiming(duration, grace, angularRate));
  }

  private static LinearRamp buildLinearRamp(
      Ramp old,
      boolean canContinue,
      VelocityStep step,
      ChassisSpeeds end,
      double startTime,
      double stepDuration) {
    double endVx = end.vxMetersPerSecond;
    double endVy = end.vyMetersPerSecond;
    double startVx = step.startVx();
    double startVy = step.startVy();
    double linearRate = Math.hypot(endVx - startVx, endVy - startVy) / stepDuration;
    double duration = stepDuration;
    boolean holding = false;

    if (canContinue
        && MathUtil.isNear(old.linear.endVx(), startVx, EPSILON)
        && MathUtil.isNear(old.linear.endVy(), startVy, EPSILON)) {
      double elapsed = startTime - old.startTime;
      startVx = old.linear.vx(elapsed);
      startVy = old.linear.vy(elapsed);
      holding =
          MathUtil.isNear(old.linear.endVx(), endVx, EPSILON)
              && MathUtil.isNear(old.linear.endVy(), endVy, EPSILON);
      if (linearRate <= EPSILON) {
        linearRate = old.linear.timing().rate();
      }
      duration =
          AxisTiming.continuousSlewDuration(
              stepDuration,
              startTime,
              holding,
              old.startTime + old.linear.timing().duration(),
              old.linear.timing().grace(),
              Math.hypot(endVx - startVx, endVy - startVy),
              linearRate);
    }

    double dx = endVx - startVx;
    double dy = endVy - startVy;
    double grace =
        AxisTiming.continuationGrace(
            duration,
            holding,
            Math.hypot(startVx, startVy),
            Math.hypot(endVx, endVy),
            dx * dx + dy * dy,
            dx * endVx + dy * endVy);
    return new LinearRamp(
        startVx, startVy, endVx, endVy, new AxisTiming(duration, grace, linearRate));
  }

  private final double massPerWheel;
  private final double[] torqueCoeffX;
  private final double[] torqueCoeffY;
  private final Feedforwards wheelForces;
  private final SwerveRequest.ApplyRobotSpeeds request =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
  final DriveRequestSamples samples = new DriveRequestSamples("timestamped_velocity_and_force");
  private final ChassisSpeeds velocity = new ChassisSpeeds();

  private final ChassisSpeeds acceleration = new ChassisSpeeds();
  private final ChassisSpeeds robotVelocity = new ChassisSpeeds();

  private volatile Ramp ramp;

  private long lastSequence = Long.MIN_VALUE;

  InterpolatedTrajectoryRequest() {
    var spec = CompTrailblazer.DRIVE_SPEC;
    var positions = spec.modulePositionsMeters();
    int count = positions.size();
    this.massPerWheel = spec.massKg() / count;
    this.torqueCoeffX = new double[count];
    this.torqueCoeffY = new double[count];
    double sumRadiusSq = 0;
    for (int i = 0; i < count; i++) {
      double rx = positions.get(i).getX();
      double ry = positions.get(i).getY();
      sumRadiusSq += rx * rx + ry * ry;
    }
    double yawTorqueScale = spec.yawMoiKgMetersSq() / sumRadiusSq;
    for (int i = 0; i < count; i++) {
      double rx = positions.get(i).getX();
      double ry = positions.get(i).getY();
      this.torqueCoeffX[i] = -ry * yawTorqueScale;
      this.torqueCoeffY[i] = rx * yawTorqueScale;
    }
    this.wheelForces = new Feedforwards(count);
    setDirect(new ChassisSpeeds());
  }

  @Override
  public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modules) {
    var active = ramp;
    active.sample(parameters.timestamp, velocity, acceleration);
    var heading = parameters.currentPose.getRotation();
    double cos = heading.getCos();
    double sin = heading.getSin();
    robotVelocity.vxMetersPerSecond =
        velocity.vxMetersPerSecond * cos + velocity.vyMetersPerSecond * sin;
    robotVelocity.vyMetersPerSecond =
        -velocity.vxMetersPerSecond * sin + velocity.vyMetersPerSecond * cos;
    robotVelocity.omegaRadiansPerSecond = velocity.omegaRadiansPerSecond;
    computeForcesInto(acceleration, cos, sin, wheelForces);
    var status =
        request
            .withSpeeds(robotVelocity)
            .withWheelForceFeedforwardsX(wheelForces.x_newtons)
            .withWheelForceFeedforwardsY(wheelForces.y_newtons)
            .apply(parameters, modules);
    double linearShift = 0;
    double angularShift = 0;
    if (active.hasStep()) {
      double plannedEnd = active.step.startTime() + active.step.durationSeconds();
      linearShift = (active.startTime + active.linear.timing().duration()) - plannedEnd;
      angularShift = (active.startTime + active.angular.timing().duration()) - plannedEnd;
    }
    samples.capture(
        parameters.timestamp,
        active.step.sequence(),
        velocity,
        acceleration,
        linearShift,
        angularShift,
        heading.getRadians(),
        status.value,
        wheelForces);
    return status;
  }

  private void computeForcesInto(
      ChassisSpeeds fieldAcceleration, double cos, double sin, Feedforwards out) {
    double fxPerWheel =
        massPerWheel
            * (fieldAcceleration.vxMetersPerSecond * cos
                + fieldAcceleration.vyMetersPerSecond * sin);
    double fyPerWheel =
        massPerWheel
            * (-fieldAcceleration.vxMetersPerSecond * sin
                + fieldAcceleration.vyMetersPerSecond * cos);
    double alpha = fieldAcceleration.omegaRadiansPerSecond;
    for (int i = 0; i < torqueCoeffX.length; i++) {
      out.x_newtons[i] = fxPerWheel + torqueCoeffX[i] * alpha;
      out.y_newtons[i] = fyPerWheel + torqueCoeffY[i] * alpha;
    }
  }

  Feedforwards forces(ChassisSpeeds fieldAcceleration, Rotation2d heading) {
    var out = new Feedforwards(torqueCoeffX.length);
    computeForcesInto(fieldAcceleration, heading.getCos(), heading.getSin(), out);
    return out;
  }

  boolean hasStep() {
    return ramp.hasStep();
  }

  void sample(double time, ChassisSpeeds v, ChassisSpeeds a) {
    ramp.sample(time, v, a);
  }

  void setDirect(ChassisSpeeds v) {
    ramp = Ramp.direct(v);
  }

  void setStep(VelocityStep step, ChassisSpeeds end) {
    setStep(step, end, step.startTime());
  }

  void setStep(VelocityStep step, ChassisSpeeds end, double publishTime) {
    double stepDuration = step.durationSeconds();
    if (stepDuration <= EPSILON) {
      setDirect(end);
      return;
    }
    if (step.sequence() <= lastSequence) {
      return;
    }
    lastSequence = step.sequence();

    double startTime =
        Double.isFinite(publishTime) ? Math.max(step.startTime(), publishTime) : step.startTime();
    var old = ramp;
    boolean canContinue = old.hasStep() && startTime >= old.startTime;

    var linear = buildLinearRamp(old, canContinue, step, end, startTime, stepDuration);
    var angular = buildAngularRamp(old, canContinue, step, end, startTime, stepDuration);
    ramp = new Ramp(step, startTime, linear, angular);
  }
}
