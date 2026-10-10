package frc.robot.testing;

import com.team581.math.MathHelpers;
import com.team581.swerve.TrailblazerDriveSource;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Test adapter with optional strict checks, independent of production robot management. */
public final class StraightLineDiagnostic implements DiagnosticRoutine {
  private final String name;
  private final StraightLineConfig config;
  private final StraightLineRoutine routine;
  private final Trailblazer trailblazer;
  private final TrailblazerDriveSource driveSource;
  private final Supplier<ChassisSpeeds> speeds;
  private final MotionAssertions assertions;
  private final boolean strict;
  private boolean enabled = true;
  private boolean started;
  private boolean stopped;
  private double startTime;
  private ChassisSpeeds requested = new ChassisSpeeds();
  private Result result;

  public StraightLineDiagnostic(
      String name,
      StraightLineConfig config,
      Trailblazer trailblazer,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds,
      boolean strict) {
    this.name = name;
    this.config = config;
    this.trailblazer = trailblazer;
    this.speeds = speeds;
    this.strict = strict;
    routine = new StraightLineRoutine(config, trailblazer, pose, speeds, () -> enabled);
    driveSource = new TrailblazerDriveSource(trailblazer, pose, speeds);
    assertions = new MotionAssertions(config.maxVelocity(), config.maxAcceleration());
    result = new Result(name, Status.RUNNING, "Waiting for first sample");
  }

  @Override
  public void abort() {
    if (!result.finished()) {
      enabled = false;
      if (started) {
        routine.beforePeriodic();
        routine.periodic();
      }
      result = new Result(name, Status.ABORTED, "Test mode disabled or interrupted");
    }
    stop();
  }

  public MotionAssertions assertions() {
    return assertions;
  }

  @Override
  public ChassisSpeeds requestedSpeeds() {
    return requested;
  }

  @Override
  public Result result() {
    return result;
  }

  public StraightLineRoutine routine() {
    return routine;
  }

  @Override
  public void stop() {
    stopped = true;
    trailblazer.clearActiveSegment();
    requested = new ChassisSpeeds();
  }

  @Override
  public void tick() {
    if (result.finished() || stopped) {
      return;
    }
    double now = Timer.getFPGATimestamp();
    var actual = speeds.get();
    if (!started) {
      started = true;
      startTime = now;
      if (strict
          && (MathHelpers.getLinearVelocity(actual) > config.stoppedVelocity()
              || !MathUtil.isNear(0, actual.omegaRadiansPerSecond, Math.toRadians(5)))) {
        result = new Result(name, Status.FAILED, "Robot must be stopped before starting");
        stop();
        return;
      }
      // First output is delayed one loop so the startup command has a measurable interval.
      if (strict) {
        try {
          assertions.start(now, actual);
        } catch (IllegalArgumentException exception) {
          result = new Result(name, Status.FAILED, exception.getMessage());
          stop();
        }
        return;
      }
    }
    routine.beforePeriodic();
    routine.periodic();
    requested = driveSource.getRequestedSpeeds();
    if (strict) {
      var failure = assertions.check(now, actual, requested);
      dev.doglog.DogLog.log(
          "Tests/Regression/PeakCommandAcceleration", assertions.peakCommandAcceleration());
      dev.doglog.DogLog.log(
          "Tests/Regression/PeakMeasuredAcceleration", assertions.peakMeasuredAcceleration());
      if (failure.isPresent()) {
        result =
            new Result(
                name,
                Status.FAILED,
                failure.orElseThrow()
                    + "; command acceleration="
                    + assertions.peakCommandAcceleration()
                    + " m/s²; measured window acceleration="
                    + assertions.peakMeasuredAcceleration()
                    + " m/s²; configured acceleration limit="
                    + config.maxAcceleration()
                    + " m/s²");
        stop();
        return;
      }
    }
    if (routine.finished()) {
      var status =
          switch (routine.getState()) {
            case PASSED -> Status.PASSED;
            case ABORTED -> Status.ABORTED;
            default -> Status.FAILED;
          };
      if (strict
          && status == Status.PASSED
          && !MathUtil.isNear(0, actual.omegaRadiansPerSecond, Math.toRadians(5))) {
        status = Status.FAILED;
        result = new Result(name, status, "Still rotating at completion");
      } else {
        result = new Result(name, status, routine.reason());
      }
      stop();
    } else {
      result =
          new Result(name, Status.RUNNING, routine.reason() + "; elapsed=" + (now - startTime));
    }
  }
}
