package frc.robot.sim;

import com.team581.autos.Point;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.imu.Imu;
import frc.robot.localization.Localization;

/**
 * Sim-only helper that fakes landing on a ball when crossing the bump. Injects pitch/roll via the
 * {@link Imu#setPitch(double)}/{@link Imu#setRoll(double)} sim hooks (no-ops on a real robot).
 */
public class StuckOnBallSim {
  private static final BooleanSubscriber ENABLED = DogLog.tunable("Sim/StuckOnBall/Enabled", true);
  private static final DoubleSubscriber BALL_X_OFFSET =
      DogLog.tunable("Sim/StuckOnBall/BallXOffset", 0.0);
  private static final DoubleSubscriber BALL_Y_OFFSET =
      DogLog.tunable("Sim/StuckOnBall/BallYOffset", 0.0);
  private static final DoubleSubscriber BEACH_RADIUS =
      DogLog.tunable("Sim/StuckOnBall/BeachRadius", 0.6);
  private static final DoubleSubscriber MAX_TILT =
      DogLog.tunable("Sim/StuckOnBall/MaxTiltDegrees", 10.0);

  // Placed at the RIGHT auto's shoot point (SHOOT_X = 13.83 in RightNormalAuto) so the auto
  // settles right on top of the ball; y uses the same -0.13 offset from the bump center
  // (BUMP_OFFSET)
  private static final Pose2d BALL_POSE_RED =
      new Pose2d(13.83, FieldUtil.RED_OUTPOST_BUMP_CENTER.getY() - 0.13, Rotation2d.kZero);

  private final Localization localization;
  private final Imu imu;

  private Translation2d lastBall = Translation2d.kZero;
  private double lastDistanceToBall = Double.POSITIVE_INFINITY;

  public StuckOnBallSim(Localization localization, Imu imu) {
    this.localization = localization;
    this.imu = imu;
  }

  /** Last ball translation computed in {@link #periodic()} (field-relative). */
  public Translation2d getBall() {
    return lastBall;
  }

  /** Last robot-to-ball distance computed in {@link #periodic()}. */
  public double getDistanceToBall() {
    return lastDistanceToBall;
  }

  public void periodic() {
    if (!RobotBase.isSimulation() || !ENABLED.get()) {
      return;
    }

    // Offsets are applied to the red-side pose before flipping for blue alliance
    Translation2d ball =
        Point.ofRed(
                new Pose2d(
                    BALL_POSE_RED.getX() + BALL_X_OFFSET.get(),
                    BALL_POSE_RED.getY() + BALL_Y_OFFSET.get(),
                    Rotation2d.kZero))
            .getPose()
            .getTranslation();

    var robotPose = localization.getPose();
    var toRobot = robotPose.getTranslation().minus(ball); // ball -> robot, field-relative
    var distance = toRobot.getNorm();
    var radius = BEACH_RADIUS.get();
    lastBall = ball;
    lastDistanceToBall = distance;
    if (distance >= radius || distance < 1e-9) {
      imu.setPitch(0.0);
      imu.setRoll(0.0);
    } else {
      var robotRelative = toRobot.rotateBy(robotPose.getRotation().unaryMinus());
      double dx = robotRelative.getX() / distance;
      double dy = robotRelative.getY() / distance;
      // Recovery math: heading = atan2(pitch, roll); recovery direction (robot-relative)
      // = (sin(heading), -cos(heading)). We want that to equal (dx, dy) - away from ball.
      double heading = Math.atan2(dx, -dy);
      double tiltDegrees = MAX_TILT.get() * (1.0 - distance / radius);
      imu.setPitch(tiltDegrees * Math.sin(heading));
      imu.setRoll(tiltDegrees * Math.cos(heading));
    }
    DogLog.log("Sim/StuckOnBall/BallPose", new Pose2d(ball, Rotation2d.kZero));
    DogLog.log("Sim/StuckOnBall/DistanceToBall", distance);
  }
}
