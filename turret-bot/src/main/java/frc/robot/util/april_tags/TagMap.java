package frc.robot.util.april_tags;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;

/**
 * Field constants and AprilTag layout for the FRC 2026 WELDED field map.
 * * Data Source: FRC2026_WELDED.fmap
 * Origin: Appears to be Center of Field (based on negative coordinates).
 */
public class TagMap {

    public static final double FIELD_LENGTH_METERS = 16.541;
    public static final double FIELD_WIDTH_METERS = 8.069;

    private static final Map<Integer, Pose2d> aprilTags = new HashMap<>();

    static {
        aprilTags.put(1, new Pose2d(11.87798, 7.42478, new Rotation2d(Math.PI)));
        aprilTags.put(2, new Pose2d(11.91542, 4.63804, new Rotation2d(Math.PI / 2)));
        aprilTags.put(3, new Pose2d(11.31186, 4.39024, new Rotation2d(Math.PI)));
        aprilTags.put(4, new Pose2d(11.31186, 4.03464, new Rotation2d(Math.PI)));
        aprilTags.put(5, new Pose2d(11.91542, 3.43124, new Rotation2d(-Math.PI / 2)));
        aprilTags.put(6, new Pose2d(11.87798, 0.64450, new Rotation2d(Math.PI)));
        
        aprilTags.put(7, new Pose2d(11.95288, 0.64450, new Rotation2d(0.0)));
        aprilTags.put(8, new Pose2d(12.27102, 3.43124, new Rotation2d(-Math.PI / 2)));
        aprilTags.put(9, new Pose2d(12.51918, 3.67904, new Rotation2d(0.0)));
        aprilTags.put(10, new Pose2d(12.51918, 4.03464, new Rotation2d(0.0)));
        aprilTags.put(11, new Pose2d(12.27102, 4.63804, new Rotation2d(Math.PI / 2)));
        aprilTags.put(12, new Pose2d(11.95288, 7.42478, new Rotation2d(0.0)));
        
        aprilTags.put(13, new Pose2d(16.53332, 7.40331, new Rotation2d(Math.PI)));
        aprilTags.put(14, new Pose2d(16.53332, 6.97151, new Rotation2d(Math.PI)));
        aprilTags.put(15, new Pose2d(16.53296, 4.32356, new Rotation2d(Math.PI)));
        aprilTags.put(16, new Pose2d(16.53296, 3.89176, new Rotation2d(Math.PI)));
        
        aprilTags.put(17, new Pose2d(4.66308, 0.64450, new Rotation2d(0.0)));
        aprilTags.put(18, new Pose2d(4.62562, 3.43124, new Rotation2d(-Math.PI / 2)));
        aprilTags.put(19, new Pose2d(5.22917, 3.67904, new Rotation2d(0.0)));
        aprilTags.put(20, new Pose2d(5.22917, 4.03464, new Rotation2d(0.0)));
        aprilTags.put(21, new Pose2d(4.62562, 4.63804, new Rotation2d(Math.PI / 2)));
        aprilTags.put(22, new Pose2d(4.66308, 7.42478, new Rotation2d(0.0)));
        
        aprilTags.put(23, new Pose2d(4.58818, 7.42478, new Rotation2d(Math.PI)));
        aprilTags.put(24, new Pose2d(4.27002, 4.63804, new Rotation2d(Math.PI / 2)));
        aprilTags.put(25, new Pose2d(4.02186, 4.39024, new Rotation2d(Math.PI)));
        aprilTags.put(26, new Pose2d(4.02186, 4.03464, new Rotation2d(Math.PI)));
        aprilTags.put(27, new Pose2d(4.27002, 3.43124, new Rotation2d(-Math.PI / 2)));
        aprilTags.put(28, new Pose2d(4.58818, 0.64450, new Rotation2d(Math.PI)));
        
        aprilTags.put(29, new Pose2d(0.00775, 0.66596, new Rotation2d(0.0)));
        aprilTags.put(30, new Pose2d(0.00775, 1.09776, new Rotation2d(0.0)));
        aprilTags.put(31, new Pose2d(0.00808, 3.74571, new Rotation2d(0.0)));
        aprilTags.put(32, new Pose2d(0.00808, 4.17751, new Rotation2d(0.0)));
    }

    /**
     * Gets the Pose2d of a specific AprilTag ID from 2026
     *
     * @param id The AprilTag ID (1-32)
     * @return The Pose2d of the tag (Translation X, Y and Rotation Z), or null if ID is invalid.
     */
    public static Pose2d getTagPose(int id) {
        return aprilTags.get(id);
    }
}