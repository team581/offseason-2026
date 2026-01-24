package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class MathHelpersTest {
  @Test
  void getClosestPointOnRectanglePerimeterTest() {
    var rectangle = new Rectangle2d(new Translation2d(0, 0), new Translation2d(4, 2));

    // Point outside
    var p1 = new Translation2d(5, 1);
    var expected1 = new Translation2d(4, 1);
    assertEquals(expected1, MathHelpers.getClosestPointOnRectanglePerimeter(p1, rectangle));

    // Point inside, closer to vertical edge
    var p2 = new Translation2d(0.5, 1);
    var expected2 = new Translation2d(0, 1);
    assertEquals(expected2, MathHelpers.getClosestPointOnRectanglePerimeter(p2, rectangle));

    // Point inside, closer to horizontal edge
    var p3 = new Translation2d(2, 0.2);
    var expected3 = new Translation2d(2, 0);
    assertEquals(expected3, MathHelpers.getClosestPointOnRectanglePerimeter(p3, rectangle));

    // Point on an edge
    var p4 = new Translation2d(4, 1);
    assertEquals(p4, MathHelpers.getClosestPointOnRectanglePerimeter(p4, rectangle));

    // Point at a corner
    var p5 = new Translation2d(4, 2);
    assertEquals(p5, MathHelpers.getClosestPointOnRectanglePerimeter(p5, rectangle));

    // Rotated rectangle
    var rotatedRectCenter = new Pose2d(2, 1, Rotation2d.fromDegrees(90));
    var rotatedRectangle = new Rectangle2d(rotatedRectCenter, 4, 2); // width 4, height 2

    // The rectangle is centered at (2,1) and rotated by 90 degrees.
    // Its body frame has x-axis along the global y-axis, and y-axis along the global -x-axis.
    // x-width is 4, so it extends 2 units along its x-axis (global +y and -y).
    // y-width is 2, so it extends 1 unit along its y-axis (global -x and +x).
    // So, in global coords, it spans from y=-1 to y=3, and from x=1 to x=3.

    var p6 = new Translation2d(1.5, 0.5); // A point inside the rotated rectangle
    var expected6 =
        new Translation2d(1, 0.5); // Closest point on perimeter is on the -x side (global)
    var result6 = MathHelpers.getClosestPointOnRectanglePerimeter(p6, rotatedRectangle);
    assertEquals(expected6.getX(), result6.getX(), 1e-9);
    assertEquals(expected6.getY(), result6.getY(), 1e-9);
  }

  @Test
  void getDriveDirectionReturnsDirectionFromStartToEnd() {
    var start = new Translation2d(0, 0);
    var end = new Translation2d(2, 0);

    var result = MathHelpers.getDriveDirection(start, end);

    // Direction from (0,0) to (2,0) should be 0 degrees (pointing right along +X)
    assertEquals(Rotation2d.kZero, result);
  }

  @Test
  void getIntersectionOnRectanglePerimeterTest() {
    var rectangle = new Rectangle2d(new Translation2d(0, 0), new Translation2d(4, 2));

    // Point outside, heading towards rectangle
    var outsidePoint = new Translation2d(5, 1);
    var outsideExpected = new Translation2d(4, 1);
    var outsideResult =
        MathHelpers.getIntersectionOnRectanglePerimeter(
            outsidePoint, rectangle, Rotation2d.fromDegrees(180));
    assertEquals(outsideExpected.getX(), outsideResult.getX(), 1e-9);
    assertEquals(outsideExpected.getY(), outsideResult.getY(), 1e-9);

    // Point inside, heading out
    var insidePoint = new Translation2d(2, 1);
    var insideExpected = new Translation2d(3, 2); // Hits the corner
    var insideResult =
        MathHelpers.getIntersectionOnRectanglePerimeter(
            insidePoint, rectangle, Rotation2d.fromDegrees(45));
    assertEquals(insideExpected.getX(), insideResult.getX(), 1e-9);
    assertEquals(insideExpected.getY(), insideResult.getY(), 1e-9);

    // Rotated rectangle
    var rotatedRectCenter = new Pose2d(5, 5, Rotation2d.fromDegrees(90));
    var rotatedRectangle = new Rectangle2d(rotatedRectCenter, 4, 2); // width 4, height 2
    var rotatedPoint = new Translation2d(5, 0); // Below rectangle
    var rotatedExpected = new Translation2d(5, 3);
    var rotatedResult =
        MathHelpers.getIntersectionOnRectanglePerimeter(
            rotatedPoint, rotatedRectangle, Rotation2d.fromDegrees(90));
    assertEquals(rotatedExpected.getX(), rotatedResult.getX(), 1e-9);
    assertEquals(rotatedExpected.getY(), rotatedResult.getY(), 1e-9);

    // No intersection case
    var noInterPoint = new Translation2d(5, 1);
    // Should return to closest point
    var noInterExpected = new Translation2d(4, 1);
    var noInterResult =
        MathHelpers.getIntersectionOnRectanglePerimeter(
            noInterPoint, rectangle, Rotation2d.fromDegrees(0));
    assertEquals(noInterExpected.getX(), noInterResult.getX(), 1e-9);
    assertEquals(noInterExpected.getY(), noInterResult.getY(), 1e-9);
  }

  @Test
  void interpolateChassisSpeedsTest() {
    var a = new ChassisSpeeds(0, 10, 0);
    var b = new ChassisSpeeds(10, 20, 0);
    var t = 0.25;
    var expected = new ChassisSpeeds(2.5, 12.5, 0);

    var result = MathHelpers.interpolate(a, b, t);

    assertEquals(expected, result);
  }

  @Test
  void roundToTest() {
    var result = MathHelpers.roundTo(123.45, 1);
    var expected = 123.5;

    assertEquals(expected, result);
  }

  @Test
  void signedSqrtTest() {
    var result = MathHelpers.signedSqrt(-36);

    assertEquals(-6, result);
  }
}
