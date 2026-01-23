package frc.robot.autos;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Points {
  NON_PROCESSOR_SIDE_START_ANGLED(
      new Point(
          new Pose2d(0,0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(0,0, Rotation2d.fromDegrees(0.0)))),

  PROCESSOR_SIDE_START_ANGLED(
      new Point(
          new Pose2d(0,0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(0,0, Rotation2d.fromDegrees(0.0)))),

  START_R1_AND_B1(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)))),
  START_R1_AND_B1_FORWARD(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),

  START_R2_AND_B2(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  START_R3_AND_B3(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  START_R3_AND_B3_LEFT_FORWARD(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),

  START_R4_AND_B4(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  START_R5_AND_B5(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  START_R6_AND_B6_FORWARD(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  START_R6_AND_B6(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)))),

  GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION(
     Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  PRE_GROUND_INTAKE_NON_PROCESSOR_SIDE_STATION(
     Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  GROUND_INTAKE_PROCESSOR_SIDE_STATION(
     Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
  PRE_GROUND_INTAKE_PROCESSOR_SIDE_STATION(
      Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),

  LOLLIPOP_2(Point.ofRed(new Pose2d(0, 0, Rotation2d.fromDegrees(180.0))));

  public final Point point;

  Points(Point point) {
    this.point = point;
  }

  public Pose2d getPose() {
    return point.getPose();
  }
}
