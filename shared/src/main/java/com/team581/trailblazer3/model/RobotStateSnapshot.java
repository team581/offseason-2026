package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;
import java.util.OptionalDouble;

public record RobotStateSnapshot(
    double poseTimestampSeconds,
    Pose2d measuredPose,
    ChassisSpeeds measuredFieldSpeeds,
    boolean localizationValid,
    boolean localizationResetOccurred,
    Optional<ChassisSpeeds> appliedFieldSpeeds,
    OptionalDouble swerveSupplyCurrentLimitAmps,
    OptionalDouble restingBatteryVoltageVolts,
    boolean enabled) {
  public static RobotStateSnapshot measured(
      double time, Pose2d pose, ChassisSpeeds speeds, boolean enabled) {
    return new RobotStateSnapshot(
        time,
        pose,
        speeds,
        true,
        false,
        Optional.empty(),
        OptionalDouble.empty(),
        OptionalDouble.empty(),
        enabled);
  }
}
