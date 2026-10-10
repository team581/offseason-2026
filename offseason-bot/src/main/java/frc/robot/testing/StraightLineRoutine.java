package frc.robot.testing;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** The same small test state machine runs against robot sensors or desktop simulation. */
public final class StraightLineRoutine extends StateMachine<StraightLineRoutine.State> {
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
  private final StraightLineConfig config;
  private final PoseErrorTolerance tolerance;
  private final Trailblazer trailblazer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final BooleanSupplier enabled;
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

  public StraightLineRoutine(
      StraightLineConfig config,
      Trailblazer trailblazer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BooleanSupplier enabled) {
    super(State.WAITING);
    this.config = config;
    tolerance = new PoseErrorTolerance(config.positionTolerance(), config.headingTolerance());
    this.trailblazer = trailblazer;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    this.enabled = enabled;
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

  public Sample sample() {
    return sample;
  }

  @Override
  protected void afterTransition(State state) {
    if (state == State.DRIVING && !started) {
      started = true;
      start = pose;
      direction = start.getRotation().plus(Rotation2d.fromDegrees(config.direction()));
      goal =
          new Pose2d(
              start.getTranslation().plus(new Translation2d(config.distance(), direction)),
              start.getRotation());
      startTime = Timer.getFPGATimestamp();
      previousTime = startTime;
      var speeds = speedsSupplier.get();
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
              Math.abs(config.distance()));
      reason = "Running";
      // Both poses are identical: this robot-relative test must not mirror with alliance.
      trailblazer.setActiveSegment(
          Trailblazer.segment(AutoPoint.of(new Point(goal, goal)))
              .withLinearConstraints(config.maxVelocity(), config.maxAcceleration())
              .untilFinished(tolerance));
      DogLog.log(LOG + "Config", config.toString());
      DogLog.log(LOG + "StartPose", start);
      DogLog.log(LOG + "GoalPose", goal);
    }
    if (finished()) {
      trailblazer.clearActiveSegment();
    }
  }

  @Override
  protected void collectInputs() {
    pose = poseSupplier.get();
    if (getState() == State.WAITING || finished()) {
      return;
    }
    var speeds = speedsSupplier.get();
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
    if (enabled.getAsBoolean()
        && (!Double.isFinite(pose.getX())
            || !Double.isFinite(pose.getY())
            || !Double.isFinite(pose.getRotation().getRadians()))) {
      reason = "Nonfinite pose data";
      return State.FAILED;
    }
    if (current == State.WAITING) {
      return enabled.getAsBoolean() ? State.DRIVING : current;
    }
    if (!enabled.getAsBoolean()) {
      reason = "Test mode disabled or interrupted";
      return State.ABORTED;
    }
    if (!Double.isFinite(sample.positionError())
        || !Double.isFinite(sample.velocity())
        || !Double.isFinite(sample.acceleration())
        || !Double.isFinite(sample.crossTrackError())
        || !Double.isFinite(angularVelocity)) {
      reason = "Nonfinite sensor data";
      return State.FAILED;
    }
    if (sample.time() >= config.timeout()) {
      reason = "Timed out before reaching and settling at the goal";
      return State.FAILED;
    }
    if (maxCrossTrackError > config.crossTrackTolerance()) {
      reason = "Exceeded cross-track tolerance";
      return State.FAILED;
    }
    boolean atPose = tolerance.atPose(goal, pose);
    if (!atPose) {
      stableSince = -1;
      return State.DRIVING;
    }
    boolean stopped =
        Math.hypot(sample.vx(), sample.vy()) <= config.stoppedVelocity()
            && Math.abs(angularVelocity) <= Math.toRadians(5);
    if (!stopped) {
      stableSince = -1;
      return State.SETTLING;
    }
    if (stableSince < 0) {
      stableSince = Timer.getFPGATimestamp();
    }
    if (Timer.getFPGATimestamp() - stableSince >= config.settleSeconds()) {
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
