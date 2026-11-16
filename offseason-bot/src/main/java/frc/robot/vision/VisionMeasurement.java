package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A field pose Spot measured.
 *
 * @param timestamp Capture time in FPGA seconds.
 * @param std Standard deviation of the position in meters, before {@link
 *     Vision#correlationFactor()}.
 * @param innovationM Distance from the odometry pose at capture.
 */
public record VisionMeasurement(Pose2d pose, double timestamp, double std, double innovationM) {}
