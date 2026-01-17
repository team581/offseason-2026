package com.team581.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheMove {
  public static Translation2d getVelocityCompensatedGoal(
      Translation2d target, ChassisSpeeds robotVelocity, double timeOfFlight) {
    // Compensated goal = real goal - (robot velocity * time of flight of ball)
    return new Translation2d(
        target.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
        target.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
  }

  public ShootOnTheMove() {}
}
