package com.team581.mechanisms.imu;

import com.team581.autos.Point;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import java.util.function.DoubleSupplier;

/**
 * Provides a pose for path following that accounts for bump crossings. When active and the robot is
 * tilted (on the bump), returns a pose projected far ahead in the crossing direction so the path
 * follower commands full output. When the robot returns to flat ground (debounced), returns the
 * real pose so normal path following resumes.
 */
public class BumpCrossingTracker {
  private static final double FLAT_DEBOUNCE_SECONDS = 0.15;
  private static final DoubleSubscriber FLAT_THRESHOLD =
      DogLog.tunable("BumpCrossing/FlatThresholdDegrees", 3.0);
  private static final DoubleSubscriber PROJECTION_DISTANCE_METERS =
      DogLog.tunable("BumpCrossing/ProjectionDistanceMeters", 10.0);

  private final Debouncer flatDebouncer =
      new Debouncer(FLAT_DEBOUNCE_SECONDS, DebounceType.kRising);
  private final DoubleSupplier tiltSupplier;

  private boolean active = false;
  private BumpCrossingDirection direction = BumpCrossingDirection.TOWARDS_DRIVER_STATION;

  public BumpCrossingTracker(DoubleSupplier tiltSupplier) {
    this.tiltSupplier = tiltSupplier;
  }

  /**
   * Get a {@link Point} adjusted for bump crossing. Use this as a pose supplier in {@code
   * AutoPoint.of(() -> tracker.getPoint(Point.ofRed(...)))}.
   *
   * @param point The base point to adjust.
   * @return The point as-is if inactive or flat, or a projected point if on the bump.
   */
  public Point getPoint(Point point) {
    Pose2d adjusted = getPose(point.getPose(), tiltSupplier.getAsDouble());
    return new Point(adjusted, adjusted);
  }

  /**
   * Get the pose to feed to the path follower.
   *
   * @param realPose The robot's actual pose from localization.
   * @param tilt The combined tilt magnitude in degrees (typically {@code Math.hypot(pitch, roll)}).
   * @return The real pose if inactive or flat, or a projected pose if on the bump.
   */
  public Pose2d getPose(Pose2d realPose, double tilt) {
    if (!active) {
      return realPose;
    }

    boolean isFlat = flatDebouncer.calculate(tilt <= FLAT_THRESHOLD.get());

    DogLog.log("BumpCrossing/Direction", direction);
    DogLog.log("BumpCrossing/IsFlatDebounced", isFlat);

    if (isFlat) {
      return realPose;
    }

    double xOffset = direction.getXSign() * PROJECTION_DISTANCE_METERS.get();

    return new Pose2d(realPose.getX() + xOffset, realPose.getY(), realPose.getRotation());
  }

  /** Whether bump crossing mode is active. */
  public boolean isActive() {
    return active;
  }

  /** Deactivate bump crossing mode. */
  public void reset() {
    active = false;
  }

  /**
   * Activate bump crossing mode. Call this when the robot is about to cross a bump.
   *
   * @param direction The direction the robot is crossing relative to its driver station.
   */
  public void setDirection(BumpCrossingDirection direction) {
    this.direction = direction;
    this.active = true;
  }
}
