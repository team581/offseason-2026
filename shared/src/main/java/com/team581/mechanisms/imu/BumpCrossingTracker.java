package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Provides a pose for path following that accounts for bump crossings. When the robot is tilted (on
 * the bump), returns a pose projected far ahead in the crossing direction so the path follower
 * commands full output. When the robot is on flat ground (debounced), returns the real target pose
 * so normal path following resumes.
 *
 * <p>The crossing direction is inferred from the robot's position relative to the target point.
 * Only points that use {@link #getPoint(Point)} are affected — regular {@code AutoPoint.ofRed()}
 * points bypass this entirely.
 */
public class BumpCrossingTracker {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.25;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 5.0);
  private static final DoubleSubscriber PROJECTION_DISTANCE_METERS =
      DogLog.tunable("BumpCrossing/ProjectionDistanceMeters", 5.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier tiltSupplier;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Consumer<Pose2d> poseResetConsumer;
  private boolean previousIsFlat = true;

  /** Latched crossing direction: +1 or -1. 0 means not currently crossing. */
  private double latchedXSign = 0;

  public BumpCrossingTracker(
      DoubleSupplier tiltSupplier,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<Pose2d> poseResetConsumer) {
    this.poseResetConsumer = poseResetConsumer;

    if (RobotBase.isSimulation()) {
      this.tiltSupplier =
          () -> {
            var pose = robotPoseSupplier.get();
            var bump = FieldUtil.getCurrentBump(pose.getTranslation());

            if (bump.isEmpty()) {
              return 0.0;
            }

            var bumpRect = bump.orElseThrow();
            double halfWidth = bumpRect.getXWidth() / 2.0;
            double distFromCenter = Math.abs(pose.getX() - bumpRect.getCenter().getX());
            double t = MathUtil.clamp(distFromCenter / halfWidth, 0.0, 1.0);

            return MathUtil.interpolate(15.0, 3.0, t);
          };
    } else {
      this.tiltSupplier = tiltSupplier;
    }
    this.robotPoseSupplier = robotPoseSupplier;
  }

  /**
   * Get a {@link Point} adjusted for bump crossing. Use this as a pose supplier in {@code
   * AutoPoint.of(() -> tracker.getPoint(Point.ofRed(...)))}.
   *
   * @param point The base target point.
   * @return The point as-is if flat, or a projected point if on the bump.
   */
  public Point getPoint(Point point) {
    return getPoint(point, null);
  }

  /**
   * Get a {@link Point} adjusted for bump crossing. Use this as a pose supplier in {@code
   * AutoPoint.of(() -> tracker.getPoint(Point.ofRed(...)))}.
   *
   * @param point The base target point.
   * @param landingPoint The point on the field where the robot is expected to land after crossing.
   *     Used to help recover pose estimation.
   * @return The point as-is if flat, or a projected point if on the bump.
   */
  public Point getPoint(Point point, @Nullable Point landingPoint) {
    double tilt = tiltSupplier.getAsDouble();
    boolean isFlat = flatDebouncer.calculate(tilt <= FLAT_THRESHOLD.get());

    DogLog.log("Imu/BumpCrossing/IsFlatDebounced", isFlat);
    DogLog.log("Imu/BumpCrossing/OriginalPoint", point.getPose());

    Pose2d targetPose = point.getPose();

    if (isFlat && previousIsFlat != isFlat && landingPoint != null) {
      // We just crossed, reset pose
      poseResetConsumer.accept(landingPoint.getPose());
    }

    previousIsFlat = isFlat;

    if (isFlat) {
      latchedXSign = 0;
      return point;
    }

    Pose2d robotPose = robotPoseSupplier.get();

    // Latch the crossing direction on the first tilted cycle so overshooting doesn't flip it.
    if (latchedXSign == 0) {
      latchedXSign = Math.signum(targetPose.getX() - robotPose.getX());
    }

    double xOffset = latchedXSign * PROJECTION_DISTANCE_METERS.get();

    Pose2d projected =
        new Pose2d(targetPose.getX() + xOffset, targetPose.getY(), targetPose.getRotation());

    DogLog.log("Imu/BumpCrossing/ProjectedPoint", projected);

    return new Point(projected, projected);
  }

  public void log() {
    DogLog.log("Imu/BumpCrossing/Tilt", tiltSupplier.getAsDouble());
  }
}
