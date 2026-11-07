package frc.robot.autos.auto_state_machines;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.autos.Point;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretConfig;
import org.junit.jupiter.api.Test;

final class NormalAutoTurretRangeTest {
  private static void assertShootingTurnHasRoom(Point goal, double startY) {
    assertShootingTurnHasRoom(goal, 13.576, startY);
  }

  private static void assertShootingTurnHasRoom(Point goal, double startX, double startY) {
    // Another hundredth of a degree must exceed the clearance at one end of the path.
    var heading = goal.redPose().getRotation();
    var further = heading.plus(Rotation2d.fromDegrees(Math.copySign(0.01, heading.getDegrees())));
    assertThat(
            NormalAutoSotm.hasClearance(new Pose2d(startX, startY, further))
                && NormalAutoSotm.hasClearance(
                    new Pose2d(goal.redPose().getTranslation(), further)))
        .as("shooting turn uses the maximum available turret travel")
        .isFalse();

    // Translation and rotation have separate constraints: check every combination of progress,
    // rather than assuming that both finish at the same time. All three cycles share this goal.
    for (int translationStep = 0; translationStep <= 40; translationStep++) {
      double progress = translationStep / 40.0;
      for (int rotationStep = 0; rotationStep <= 40; rotationStep++) {
        var redPose =
            new Pose2d(
                MathUtil.interpolate(startX, goal.redPose().getX(), progress),
                MathUtil.interpolate(startY, goal.redPose().getY(), progress),
                Rotation2d.kZero.interpolate(goal.redPose().getRotation(), rotationStep / 40.0));
        var point = Point.ofRed(redPose);
        assertTurretHasRoom(point.redPose(), FieldUtil.HUB_POSE.redPose());
        assertTurretHasRoom(point.bluePose(), FieldUtil.HUB_POSE.bluePose());
      }
    }
  }

  private static void assertTurretHasRoom(Pose2d robot, Pose2d hub) {
    // Check the raw aiming angle before clamping so an unreachable target cannot pass this test.
    double angle = TurretCalculator.calculateTurretAimingAngle(robot, hub.getTranslation());
    double unwrapped = MathUtil.inputModulus(angle, -360.0, 0.0);
    assertThat(unwrapped)
        .as("turret angle at robot pose %s", robot)
        .isBetween(
            TurretConfig.MIN_ANGLE + NormalAutoSotm.HARD_STOP_CLEARANCE_DEGREES - 1e-8,
            TurretConfig.MAX_ANGLE - NormalAutoSotm.HARD_STOP_CLEARANCE_DEGREES + 1e-8);
  }

  @Test
  void leftBumpShootingTurnKeepsHubWithinTurretTravel() {
    assertShootingTurnHasRoom(
        LeftBumpAuto.SHOOT_ON_THE_MOVE_GOAL,
        LeftBumpAuto.SHOOT_X,
        FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.13);
  }

  @Test
  void leftDepotShootingLegsKeepHubWithinTurretTravel() {
    var start =
        new Pose2d(
            LeftBumpDepotAuto.RETURN_BUMP_LANDING_X,
            FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.13,
            Rotation2d.kZero);
    for (int step = 0; step <= 40; step++) {
      double progress = step / 40.0;
      for (int turn = 0; turn <= 40; turn++) {
        var approach =
            new Pose2d(
                start
                    .getTranslation()
                    .interpolate(
                        LeftBumpDepotAuto.DEPOT_APPROACH.redPose().getTranslation(), progress),
                start
                    .getRotation()
                    .interpolate(
                        LeftBumpDepotAuto.DEPOT_APPROACH.redPose().getRotation(), turn / 40.0));
        var point = Point.ofRed(approach);
        assertTurretHasRoom(point.redPose(), FieldUtil.HUB_POSE.redPose());
        assertTurretHasRoom(point.bluePose(), FieldUtil.HUB_POSE.bluePose());
      }
      var returning =
          Point.ofRed(
              LeftBumpDepotAuto.DEPOT_FRONT
                  .redPose()
                  .interpolate(LeftBumpDepotAuto.FINAL_SHOOT_GOAL.redPose(), progress));
      assertTurretHasRoom(returning.redPose(), FieldUtil.HUB_POSE.redPose());
      assertTurretHasRoom(returning.bluePose(), FieldUtil.HUB_POSE.bluePose());
    }
  }

  @Test
  void leftShootingTurnKeepsHubWithinTurretTravel() {
    assertShootingTurnHasRoom(
        LeftNormalAuto.SHOOT_ON_THE_MOVE_GOAL, FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.13);
  }

  @Test
  void rightBumpShootingTurnKeepsHubWithinTurretTravel() {
    assertShootingTurnHasRoom(
        RightBumpAuto.SHOOT_ON_THE_MOVE_GOAL,
        RightBumpAuto.SHOOT_X,
        FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() - 0.13);
  }

  @Test
  void rightShootingTurnKeepsHubWithinTurretTravel() {
    assertShootingTurnHasRoom(
        RightNormalAuto.SHOOT_ON_THE_MOVE_GOAL, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() - 0.13);
  }
}
