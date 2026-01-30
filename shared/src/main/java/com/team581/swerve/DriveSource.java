package com.team581.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.jspecify.annotations.Nullable;

/**
 * A drive source is an abstraction over something that outputs requested chassis speeds. These
 * speeds may be followed exactly (ex. driving normally in teleop) or might be processed (ex.
 * snapping to face a specific angle).
 *
 * <p>Realistically, the only implementations of this are {@link XboxControllerDriveSource} and
 * {@link TrailblazerDriveSource}.
 */
public interface DriveSource {
  DriveSourceType getDriveSourceType();

  default ChassisSpeeds getRequestedSpeeds() {
    return getRequestedSpeeds(null);
  }

  ChassisSpeeds getRequestedSpeeds(@Nullable Rotation2d snapAngle);
}
