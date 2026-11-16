package frc.robot.autos;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;

/** Shared 2026 mechanism actions and transition conditions for autonomous state machines. */
public final class AutoActions {
  public static final double DEFAULT_MIN_SCORE_FEED_SECONDS = 0.75;
  public static final double DEFAULT_SHOOT_TIMEOUT_SECONDS = 3.0;

  /** Returns true once scoring has fed at least the minimum duration and emptied, or timed out. */
  public static boolean finishedShooting(TrailblazerAuto<?> auto) {
    return finishedShooting(auto, DEFAULT_MIN_SCORE_FEED_SECONDS, DEFAULT_SHOOT_TIMEOUT_SECONDS);
  }

  /** Returns true once scoring has fed for {@code minFeedSeconds} and emptied, or timed out. */
  public static boolean finishedShooting(
      TrailblazerAuto<?> auto, double minFeedSeconds, double maxTimeoutSeconds) {
    return (auto.timeout(minFeedSeconds)
            && auto.robot.getState() == RobotState.SCORE
            && !auto.robot.hopperManager.isShooting())
        || auto.timeout(maxTimeoutSeconds);
  }

  /** Homes deploy and shooter hood at autonomous startup. */
  public static void homeSubsystems(RobotManager robot) {
    robot.homeDeployInAutoRequest();
    robot.homeShooterHoodRequest();
  }

  /** Prepares subsystems and shooter RPM for a Hub shot at {@code hubDistanceMeters}. */
  public static void prepareForBumpShot(RobotManager robot, double hubDistanceMeters) {
    robot.cancelIntakeRequest();
    robot.powerManager.idleRequest();
    robot.shooter.prepareScoreRequest(hubDistanceMeters);
  }

  /** Prepares subsystems and shooter RPM for a Hub shot from {@code blueShootPosition}. */
  public static void prepareForBumpShot(RobotManager robot, Translation2d blueShootPosition) {
    prepareForBumpShot(robot, blueShootPosition.getDistance(FieldUtil.BLUE_HUB_TRANSLATION));
  }

  /** Homes mechanisms, enables auto intaking, and prioritizes flywheel/intake power on cycle 1. */
  public static void startFirstIntakeCycle(RobotManager robot) {
    homeSubsystems(robot);
    robot.intakeAutoRequest();
    robot.powerManager.firstAutoSegmentRequest();
  }

  /** Resets robot manager to idle with safe stow enabled before driving out for the next cycle. */
  public static void startNextCycle(RobotManager robot) {
    robot.idleRequest();
    robot.hopperManager.setWantsSafeStow(true);
  }

  /** Stops intaking and requests full Hub scoring alignment and feed. */
  public static void startScoring(RobotManager robot) {
    robot.cancelIntakeRequest();
    robot.prepareScoreRequest();
  }

  private AutoActions() {}
}
