package frc.robot.testing;

import com.team581.swerve.TrailblazerDriveSource;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

/** Runs the straight-line routine and exposes its drive command and result. */
public final class StraightLineDiagnostic implements DiagnosticRoutine {
  private final String name;
  private final StraightLineRoutine routine;
  private final Trailblazer trailblazer;
  private final TrailblazerDriveSource driveSource;
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
      Supplier<ChassisSpeeds> speeds) {
    this.name = name;
    this.trailblazer = trailblazer;
    routine = new StraightLineRoutine(config, trailblazer, pose, speeds, () -> enabled);
    driveSource = new TrailblazerDriveSource(trailblazer, pose, speeds);
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
    if (!started) {
      started = true;
      startTime = now;
    }
    routine.beforePeriodic();
    routine.periodic();
    requested = driveSource.getRequestedSpeeds();
    if (routine.finished()) {
      var status =
          switch (routine.getState()) {
            case PASSED -> Status.PASSED;
            case ABORTED -> Status.ABORTED;
            default -> Status.FAILED;
          };
      result = new Result(name, status, routine.reason());
      stop();
    } else {
      result =
          new Result(name, Status.RUNNING, routine.reason() + "; elapsed=" + (now - startTime));
    }
  }
}
