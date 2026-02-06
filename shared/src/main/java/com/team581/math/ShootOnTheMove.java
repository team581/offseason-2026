package com.team581.math;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheMove {
  private final InterpolatingDoubleTreeMap distanceToTimeOfFlight;

  public ShootOnTheMove(InterpolatingDoubleTreeMap distanceToTimeOfFlight) {
    this.distanceToTimeOfFlight = distanceToTimeOfFlight;
  }

  public Translation2d getVelocityCompensatedGoal(
      Translation2d robot, Translation2d target, ChassisSpeeds robotVelocity) {
    var timeOfFlight = distanceToTimeOfFlight.get(robot.getDistance(target));

    // Compensated goal = real goal - (robot velocity * time of flight of ball)
    return new Translation2d(
        target.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlight),
        target.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlight));
  }
}
