package com.team581.localization;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleSubscriber;

public class TrustFactor {
  private static final DoubleSubscriber DISTANCE_TRAVELLED_SCALAR =
      DogLog.tunable("TrustFactor/DistanceTravelledScalar", 1.0);
  private static final DoubleSubscriber POST_COLLISION_ADDITION =
      DogLog.tunable("TrustFactor/PostCollisionAddition", 5.0);
  private static final DoubleSubscriber TRUSTWORTHY_THRESHOLD =
      DogLog.tunable("TrustFactor/TrustworthyThreshold", 1.0);
  private static final DoubleSubscriber TRUST_FACTOR_TAG_SEEN_DENOMINATOR =
      DogLog.tunable("TrustFactor/TrustFactorTagSeenDenominator", 3.0);
  private static final DoubleSubscriber TRUST_FACTOR_TAG_SEEN_MAX =
      DogLog.tunable("TrustFactor/TrustFactorTagSeenMax", 5.0);
  private double trustFactor = 0.0;
  private double metersTravelledSinceLastCheck = 0.0;
  private Pose2d lastCheckedPose = Pose2d.kZero;

  public double get() {
    return trustFactor;
  }

  public boolean isTrustworthy() {
    return trustFactor <= TRUSTWORTHY_THRESHOLD.get();
  }

  public void tagSeen() {
    trustFactor =
        Math.min(
            trustFactor / TRUST_FACTOR_TAG_SEEN_DENOMINATOR.get(), TRUST_FACTOR_TAG_SEEN_MAX.get());
  }

  public void update(Pose2d robotPose, boolean collisionDetected) {
    metersTravelledSinceLastCheck =
        lastCheckedPose.getTranslation().getDistance(robotPose.getTranslation());

    trustFactor += metersTravelledSinceLastCheck * DISTANCE_TRAVELLED_SCALAR.get();
    lastCheckedPose = robotPose;

    if (collisionDetected) {
      trustFactor += POST_COLLISION_ADDITION.get();
    }
  }
}
