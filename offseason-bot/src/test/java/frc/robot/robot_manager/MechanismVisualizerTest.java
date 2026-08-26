package frc.robot.robot_manager;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter_hood.ShooterHoodConfig;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class MechanismVisualizerTest {
  private static final double DELTA = 1e-9;

  private static boolean isVisible(Pose3d pose) {
    return pose.getZ() > -1.0;
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void hoodIsFlatAtItsHorizontalAngleAndRaisesItsRearForPositiveExtension() {
    var flatPoses =
        MechanismVisualizer.buildComponentPoses(
            RobotState.SCORE, 0.0, ShooterHoodConfig.ANGLE_FROM_HORIZONTAL, 0.0);
    var raisedPoses =
        MechanismVisualizer.buildComponentPoses(
            RobotState.SCORE, 0.0, ShooterHoodConfig.ANGLE_FROM_HORIZONTAL + 30.0, 0.0);
    var flatHood = flatPoses[0];
    var raisedHood = raisedPoses[0];

    assertEquals(flatHood.getTranslation(), raisedHood.getTranslation());
    assertEquals(0.0, flatHood.getRotation().getY(), DELTA);

    var rearOfRaisedHood =
        new Translation3d(-Units.inchesToMeters(6.0), 0.0, 0.0)
            .rotateBy(raisedHood.getRotation())
            .plus(raisedHood.getTranslation());
    assertEquals(Units.inchesToMeters(3.0), rearOfRaisedHood.getZ() - raisedHood.getZ(), DELTA);
  }

  @Test
  void onlyScoreAndFeedUseGreenComponents() {
    for (var state : RobotState.values()) {
      var poses =
          MechanismVisualizer.buildComponentPoses(
              state, 0.0, ShooterHoodConfig.ANGLE_FROM_HORIZONTAL, 0.0);
      var shouldUseGreen = state == RobotState.SCORE || state == RobotState.FEED;

      assertEquals(shouldUseGreen, isVisible(poses[0]));
      assertEquals(shouldUseGreen, isVisible(poses[1]));
      assertEquals(!shouldUseGreen, isVisible(poses[3]));
      assertEquals(!shouldUseGreen, isVisible(poses[4]));

      for (var componentIndex : new int[] {0, 1, 3, 4}) {
        assertNotEquals(Pose3d.kZero, poses[componentIndex]);
      }
    }
  }

  @Test
  void placesTurretAndHoodHingeAtConfiguredOffsets() {
    var poses =
        MechanismVisualizer.buildComponentPoses(
            RobotState.SCORE, 90.0, ShooterHoodConfig.ANGLE_FROM_HORIZONTAL, 0.0);
    var turretPose = poses[1];
    var hoodPose = poses[0];

    assertEquals(Units.inchesToMeters(8.0), turretPose.getX(), DELTA);
    assertEquals(Units.inchesToMeters(8.0), turretPose.getY(), DELTA);
    assertEquals(Units.inchesToMeters(14.0), turretPose.getZ(), DELTA);
    assertEquals(Units.inchesToMeters(8.0), hoodPose.getX(), DELTA);
    assertEquals(Units.inchesToMeters(11.0), hoodPose.getY(), DELTA);
    assertEquals(Units.inchesToMeters(20.0), hoodPose.getZ(), DELTA);
  }
}
