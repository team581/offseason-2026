package com.team581.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class TrustFactor {
  private static final double DISTANCE_TRAVELLED_SCALAR = 1.0;
  private static final double POST_COLLISION_ADDITION = 5.0;

  private static final double TRUSTWORTHY_THRESHOLD = 1.0;

  private static final double TRUST_FACTOR_TAG_SEEN_DENOMINATOR = 3.0;
  private static final double TRUST_FACTOR_TAG_SEEN_MAX = 5.0;

  private double trustFactor = 0.0;
  private double metersTravelledSinceLastCheck = 0.0;
  private Pose2d lastCheckedPose = Pose2d.kZero;

  public double get() {
    return trustFactor;
  }

  public boolean isTrustworthy() {
    return trustFactor <= TRUSTWORTHY_THRESHOLD;
  }

  public void tagSeen() {
    trustFactor =
        Math.min(trustFactor / TRUST_FACTOR_TAG_SEEN_DENOMINATOR, TRUST_FACTOR_TAG_SEEN_MAX);
  }

  public void update(Pose2d robotPose, boolean collisionDetected) {
    metersTravelledSinceLastCheck =
        lastCheckedPose.getTranslation().getDistance(robotPose.getTranslation());

    trustFactor += metersTravelledSinceLastCheck * DISTANCE_TRAVELLED_SCALAR;
    lastCheckedPose = robotPose;

    if (collisionDetected) {
      trustFactor += POST_COLLISION_ADDITION;
    }
  }
}
