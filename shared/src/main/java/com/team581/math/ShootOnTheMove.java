package com.team581.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class ShootOnTheMove {
  public static Pose2d getVelocityCompensatedGoal(
      Pose2d target, ChassisSpeeds robotVelocity, double timeOfFlight) {
    // Compensated goal = real goal + (robot velocity * time of flight of ball)
    return new Pose2d(
        target.getX() + (robotVelocity.vxMetersPerSecond * timeOfFlight),
        target.getY() + (robotVelocity.vyMetersPerSecond * timeOfFlight),
        Rotation2d.kZero);
  }

  public ShootOnTheMove() {}
}
