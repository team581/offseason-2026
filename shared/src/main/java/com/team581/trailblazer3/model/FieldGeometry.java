package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record FieldGeometry(double lengthMeters, double widthMeters, FieldSymmetry symmetry) {
  public FieldGeometry {
    requireNonNull(symmetry);
    checkArgument(
        Double.isFinite(lengthMeters)
            && Double.isFinite(widthMeters)
            && lengthMeters > 0
            && widthMeters > 0,
        "Invalid field dimensions");
  }

  public Translation2d flipPoint(Translation2d point) {
    return new Translation2d(
        lengthMeters - point.getX(),
        symmetry == FieldSymmetry.MIRROR_X ? point.getY() : widthMeters - point.getY());
  }

  public Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
        flipPoint(pose.getTranslation()),
        symmetry == FieldSymmetry.MIRROR_X
            ? Rotation2d.kPi.minus(pose.getRotation())
            : pose.getRotation().plus(Rotation2d.kPi));
  }
}
