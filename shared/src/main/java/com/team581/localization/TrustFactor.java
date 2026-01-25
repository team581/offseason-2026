package com.team581.localization;

import edu.wpi.first.math.geometry.Pose2d;

public class TrustFactor {
  private static final double MIN_TRUST_FACTOR = 0.01;

  private double trustFactor = 0.1;
  private double metersTravelledSinceLastCheck = 0.0;
  private Pose2d lastCheckedPose = Pose2d.kZero;

  public double get() {
    return trustFactor;
  }

  public void update(Pose2d robotPose) {
    metersTravelledSinceLastCheck =
        lastCheckedPose.getTranslation().getDistance(robotPose.getTranslation());
    trustFactor += Math.max(MIN_TRUST_FACTOR, trustFactor * metersTravelledSinceLastCheck * 0.001);
  }

  public void tagSeen(Pose2d robotPose) {
    trustFactor = 0.1;
    lastCheckedPose = robotPose;
  }
}
