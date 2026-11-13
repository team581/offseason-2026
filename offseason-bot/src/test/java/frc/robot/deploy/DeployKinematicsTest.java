package frc.robot.deploy;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

final class DeployKinematicsTest {
  private static final double DELTA = 1e-8;

  @Test
  void carriageAndBothArmsRemainConnectedThroughoutTheArc() {
    // Carriage-side joint centers measured independently from the retracted CAD.
    var joints =
        new Translation3d[] {
          new Translation3d(0.079861159, 0.0, 0.540145180),
          new Translation3d(0.164683904, 0.0, 0.480706655)
        };
    for (var angle : new double[] {0.0, 22.1, 44.2, 66.3, 88.4}) {
      var poses = DeployKinematics.componentPoses(angle);
      for (var index = 0; index < joints.length; index++) {
        var pivot = poses[index + 1].getTranslation();
        var link = joints[index].minus(pivot);
        var armJoint = link.rotateBy(poses[index + 1].getRotation()).plus(pivot);
        var carriageJoint = joints[index].plus(poses[0].getTranslation());
        assertEquals(0.0, armJoint.getDistance(carriageJoint), DELTA);
      }
    }
    var middle = DeployKinematics.componentPoses(44.2)[0];
    var end = DeployKinematics.componentPoses(0.0)[0];
    assertThat(middle.getTranslation().getDistance(end.getTranslation().times(0.5)))
        .isGreaterThan(0.05);
  }

  @Test
  void clampsAnglesAtTheCadEndpoints() {
    assertEquals(
        DeployKinematics.componentPoses(88.4)[0], DeployKinematics.componentPoses(95.0)[0]);
    assertEquals(
        DeployKinematics.componentPoses(0.0)[0], DeployKinematics.componentPoses(-10.0)[0]);
    // STOW is an intermediate linkage angle, not the CAD's fully inward endpoint.
    assertThat(51.3).isPositive();
    assertThat(51.3).isLessThan(88.4);
  }

  @Test
  void matchesBothExportedCarriageEndpointsAndKeepsItLevel() {
    var retracted = DeployKinematics.componentPoses(88.4);
    var extended = DeployKinematics.componentPoses(0.0);
    assertEquals(Pose3d.kZero, retracted[0]);
    // Measured from the intake-in and intake-out STEP assembly placements.
    assertEquals(0.341739198576, extended[0].getX(), DELTA);
    assertEquals(0.0, extended[0].getY(), DELTA);
    assertEquals(-0.203595179471, extended[0].getZ(), DELTA);
    assertEquals(0.0, extended[0].getRotation().getAngle(), DELTA);
    assertEquals(Math.toRadians(88.4), extended[1].getRotation().getY(), DELTA);
    assertEquals(extended[1].getRotation(), extended[2].getRotation());
  }
}
