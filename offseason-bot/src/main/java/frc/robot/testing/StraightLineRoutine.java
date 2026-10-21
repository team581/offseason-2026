package frc.robot.testing;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/** Drives to a relative goal and checks position, heading, drift, and settling. */
public final class StraightLineRoutine extends TestManager.Test<StraightLineRoutine.State> {
  public record Sample(
      double time,
      Pose2d pose,
      double vx,
      double vy,
      double velocity,
      double acceleration,
      double crossTrackError,
      double positionError) {}

  public enum State {
    WAITING,
    DRIVING,
    SETTLING,
    PASSED,
    FAILED,
    ABORTED
  }

  private static final String LOG = "Tests/StraightLine/";
  public static final double DISTANCE = 2.0;
  public static final double MAX_VELOCITY = 1.0;
  public static final double MAX_ACCELERATION = 0.75;
  public static final double TIMEOUT = 10.0;
  private static final double POSITION_TOLERANCE = 0.10;
  private static final double HEADING_TOLERANCE = 3.0;
  private static final double STOPPED_VELOCITY = 0.10;
  private static final double SETTLE_SECONDS = 0.25;
  private static final double CROSS_TRACK_TOLERANCE = 0.20;
  private final PoseErrorTolerance tolerance =
      new PoseErrorTolerance(POSITION_TOLERANCE, HEADING_TOLERANCE);
  private final TestManager manager;
  private final Trailblazer trailblazer;
  private Pose2d pose = Pose2d.kZero;
  private Pose2d start = Pose2d.kZero;
  private Pose2d goal = Pose2d.kZero;
  private Rotation2d direction = Rotation2d.kZero;
  private double startTime;
  private boolean started;
  private double previousTime;
  private double previousVelocity;
  private double angularVelocity;
  private double stableSince = -1;
  private double peakVelocity;
  private double peakAcceleration;
  private double maxCrossTrackError;
  private String reason = "Waiting for test mode enable";
  private Sample sample = new Sample(0, Pose2d.kZero, 0, 0, 0, 0, 0, 0);

  public StraightLineRoutine(TestManager manager) {
    super(State.WAITING);
    this.manager = manager;
    trailblazer = manager.trailblazer;
  }

  @Override
  public void abort() {
    if (!finished()) {
      reason = "Test mode disabled or interrupted";
      setStateFromRequest(State.ABORTED);
    }
  }

  public boolean finished() {
    return getState() == State.PASSED || getState() == State.FAILED || getState() == State.ABORTED;
  }

  public double maxCrossTrackError() {
    return maxCrossTrackError;
  }

  public double peakAcceleration() {
    return peakAcceleration;
  }

  public double peakVelocity() {
    return peakVelocity;
  }

  public String reason() {
    return reason;
  }

  @Override
  public TestManager.Result result() {
    var status =
        switch (getState()) {
          case PASSED -> TestManager.Status.PASSED;
          case FAILED -> TestManager.Status.FAILED;
          case ABORTED -> TestManager.Status.ABORTED;
          default -> TestManager.Status.RUNNING;
        };
    return new TestManager.Result("StraightLine", status, reason);
  }

  public Sample sample() {
    return sample;
  }

  @Override
  protected void afterTransition(State state) {
    if (state == State.DRIVING && !started) {
      started = true;
      start = pose;
      direction = start.getRotation();
      goal =
          new Pose2d(
              start.getTranslation().plus(new Translation2d(DISTANCE, direction)),
              start.getRotation());
      startTime = Timer.getFPGATimestamp();
      previousTime = startTime;
      var speeds = manager.speeds;
      previousVelocity =
          speeds.vxMetersPerSecond * direction.getCos()
              + speeds.vyMetersPerSecond * direction.getSin();
      sample =
          new Sample(
              0,
              start,
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              previousVelocity,
              0,
              0,
              Math.abs(DISTANCE));
      reason = "Running";
      // Use the same goal for both alliances so the test follows the starting robot heading.
      trailblazer.setActiveSegment(
          Trailblazer.segment(AutoPoint.of(new Point(goal, goal)))
              .withLinearConstraints(MAX_VELOCITY, MAX_ACCELERATION)
              .untilFinished(tolerance));
      DogLog.log(LOG + "MaxVelocity", MAX_VELOCITY);
      DogLog.log(LOG + "MaxAcceleration", MAX_ACCELERATION);
      DogLog.log(LOG + "StartPose", start);
      DogLog.log(LOG + "GoalPose", goal);
    }
    if (finished()) {
      trailblazer.clearActiveSegment();
    }
  }

  @Override
  protected void collectInputs() {
    pose = manager.pose;
    if (getState() == State.WAITING || finished()) {
      return;
    }
    var speeds = manager.speeds;
    angularVelocity = speeds.omegaRadiansPerSecond;
    double now = Timer.getFPGATimestamp();
    double velocity =
        speeds.vxMetersPerSecond * direction.getCos()
            + speeds.vyMetersPerSecond * direction.getSin();
    double acceleration =
        now > previousTime ? (velocity - previousVelocity) / (now - previousTime) : 0;
    var displacement = pose.getTranslation().minus(start.getTranslation());
    double crossTrack =
        Math.abs(
            -displacement.getX() * direction.getSin() + displacement.getY() * direction.getCos());
    sample =
        new Sample(
            now - startTime,
            pose,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            velocity,
            acceleration,
            crossTrack,
            pose.getTranslation().getDistance(goal.getTranslation()));
    peakVelocity = Math.max(peakVelocity, Math.hypot(sample.vx(), sample.vy()));
    peakAcceleration = Math.max(peakAcceleration, Math.abs(acceleration));
    maxCrossTrackError = Math.max(maxCrossTrackError, crossTrack);
    previousTime = now;
    previousVelocity = velocity;
  }

  @Override
  protected State getNextState(State current) {
    if (finished()) {
      return current;
    }
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())
        || !Double.isFinite(manager.speeds.vxMetersPerSecond)
        || !Double.isFinite(manager.speeds.vyMetersPerSecond)
        || !Double.isFinite(manager.speeds.omegaRadiansPerSecond)) {
      reason = "Nonfinite sensor data";
      return State.FAILED;
    }
    if (current == State.WAITING) {
      return State.DRIVING;
    }
    if (!Double.isFinite(sample.positionError())
        || !Double.isFinite(sample.velocity())
        || !Double.isFinite(sample.acceleration())
        || !Double.isFinite(sample.crossTrackError())
        || !Double.isFinite(angularVelocity)) {
      reason = "Nonfinite sensor data";
      return State.FAILED;
    }
    if (sample.time() >= TIMEOUT) {
      reason = "Timed out before reaching and settling at the goal";
      return State.FAILED;
    }
    if (maxCrossTrackError > CROSS_TRACK_TOLERANCE) {
      reason = "Exceeded cross-track tolerance";
      return State.FAILED;
    }
    boolean atPose = tolerance.atPose(goal, pose);
    if (!atPose) {
      stableSince = -1;
      return State.DRIVING;
    }
    boolean stopped =
        Math.hypot(sample.vx(), sample.vy()) <= STOPPED_VELOCITY
            && MathUtil.isNear(0, angularVelocity, Math.toRadians(5));
    if (!stopped) {
      stableSince = -1;
      return State.SETTLING;
    }
    if (stableSince < 0) {
      stableSince = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - stableSince >= SETTLE_SECONDS) {
      reason = "Reached goal and settled within tolerances";
      return State.PASSED;
    }
    return State.SETTLING;
  }

  @Override
  protected void whileInState(State state) {
    DogLog.log(LOG + "State", state.toString());
    DogLog.log(LOG + "Reason", reason);
    DogLog.log(LOG + "ElapsedSeconds", sample.time());
    DogLog.log(LOG + "Pose", pose);
    DogLog.log(LOG + "VelocityMetersPerSecond", sample.velocity());
    DogLog.log(LOG + "AccelerationMetersPerSecondSquared", sample.acceleration());
    DogLog.log(LOG + "CrossTrackErrorMeters", sample.crossTrackError());
    DogLog.log(LOG + "PositionErrorMeters", sample.positionError());
    DogLog.log(LOG + "PeakVelocityMetersPerSecond", peakVelocity);
    DogLog.log(LOG + "PeakAccelerationMetersPerSecondSquared", peakAcceleration);
    DogLog.log(LOG + "MaxCrossTrackErrorMeters", maxCrossTrackError);
    DogLog.log(LOG + "Passed", state == State.PASSED);
  }
}
