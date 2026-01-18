package com.team581.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
  private static final double EXTRA_NEUTRAL_ZONE_THRESHOLD = 0.5;
  // In meters
  public static double FIELD_LENGTH = 16.540988;
  public static double FIELD_WIDTH = 8.069326;

  public static Point HUB_POSE = Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
  public static Point FEED_1_POSE = Point.ofRed(new Pose2d(15.75, 0.75, Rotation2d.kZero));
  public static Point FEED_2_POSE = Point.ofRed(new Pose2d(15.75, 7.25, Rotation2d.kZero));

  // calculations
  private static final double fieldLengthX = Units.inchesToMeters(651.22);
  private static final double fieldWidthY = Units.inchesToMeters(317.69);

  private static final double redRobotStartingLineX = Units.inchesToMeters(156.61);
  private static final double blueRobotStartingLineX = fieldLengthX - redRobotStartingLineX;

  private static final double redTrenchX = Units.inchesToMeters(182.11);
  private static final double blueTrenchX = fieldLengthX - redTrenchX;

  private static final double trenchToRobotStartingLineDistanceX =
      redTrenchX - redRobotStartingLineX;

  private static final double redBoxCoordinateX =
      trenchToRobotStartingLineDistanceX + redRobotStartingLineX;
  private static final double blueBoxCoordinateX = blueTrenchX - trenchToRobotStartingLineDistanceX;

  private static final double trenchLengthY = Units.inchesToMeters(25.62 * 2);
  private static final double oppositeTrenchCoordinateY = fieldWidthY - trenchLengthY;
  // end of calculations
  public static final Rectangle2d RED_LEFT_UNSAFE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(redRobotStartingLineX, fieldWidthY),
          new Translation2d(redBoxCoordinateX, oppositeTrenchCoordinateY));
  public static Rectangle2d RED_RIGHT_UNSAFE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(redRobotStartingLineX, trenchLengthY),
          new Translation2d(redBoxCoordinateX, 0.0));

  public static Rectangle2d BLUE_LEFT_UNSAFE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(blueBoxCoordinateX, fieldWidthY),
          new Translation2d(blueRobotStartingLineX, oppositeTrenchCoordinateY));
  public static Rectangle2d BLUE_RIGHT_UNSAFE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(blueBoxCoordinateX, trenchLengthY),
          new Translation2d(blueRobotStartingLineX, 0.0));

  public static final List<Rectangle2d> TRENCH_BOXES =
      List.of(
          FieldUtil.BLUE_LEFT_UNSAFE_TRENCH_BOX,
          FieldUtil.BLUE_RIGHT_UNSAFE_TRENCH_BOX,
          FieldUtil.RED_LEFT_UNSAFE_TRENCH_BOX,
          FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX);

  public static Rectangle2d getLeftUnsafeTrenchBox() {
    return FmsUtil.isRedAlliance()
        ? FieldUtil.RED_LEFT_UNSAFE_TRENCH_BOX
        : FieldUtil.BLUE_LEFT_UNSAFE_TRENCH_BOX;
  }

  public static Rectangle2d getRightUnsafeTrenchBox() {
    return FmsUtil.isRedAlliance()
        ? FieldUtil.RED_RIGHT_UNSAFE_TRENCH_BOX
        : FieldUtil.BLUE_RIGHT_UNSAFE_TRENCH_BOX;
  }

  public static Translation2d getHubPose() {
    return FmsUtil.isRedAlliance()
        ? HUB_POSE.redPose().getTranslation()
        : HUB_POSE.bluePose().getTranslation();
  }

  public static Translation2d getFeed1Pose() {
    return FmsUtil.isRedAlliance()
        ? FEED_1_POSE.redPose().getTranslation()
        : FEED_1_POSE.bluePose().getTranslation();
  }

  public static Translation2d getFeed2Pose() {
    return FmsUtil.isRedAlliance()
        ? FEED_2_POSE.redPose().getTranslation()
        : FEED_2_POSE.bluePose().getTranslation();
  }

  // TODO: Make smarter for different rotations (would need to store bumper size)
  public static boolean isRobotInAllianceZone(Pose2d robot) {
    var goalX = getHubPose().getX();
    if (FmsUtil.isRedAlliance()) {
      if (robot.getX() > goalX + EXTRA_NEUTRAL_ZONE_THRESHOLD) {
        return true;
      }
      return false;
    }
    if (robot.getX() < goalX - EXTRA_NEUTRAL_ZONE_THRESHOLD) {
      return true;
    }
    return false;
  }
}
