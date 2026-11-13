package frc.robot.testing;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class StraightLineTestTest {
  private Pose2d pose = new Pose2d(2, 2, Rotation2d.kZero);
  private ChassisSpeeds speeds = new ChassisSpeeds();

  private StraightLineTest routine;
  private Trailblazer trailblazer;

  private StraightLineTest routine() {
    routine =
        new StraightLineTest(trailblazer, () -> pose, () -> speeds, ChassisSpeeds::new, () -> {});
    tick(0.02);
    return routine;
  }

  private void tick(double seconds) {
    SimHooks.stepTimingAsync(seconds);
    routine.beforePeriodic();
    routine.periodic();
  }

  @Test
  void abortStopsMotionAndPreservesTheResult() {
    var routine = routine();
    routine.abort();
    assertThat(routine.result().status()).isEqualTo(TestStatus.ABORTED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isEqualTo(0);
    tick(0.02);
    assertThat(routine.result().status()).isEqualTo(TestStatus.ABORTED);
  }

  @Test
  void goalFollowsStartingRobotHeading() {
    pose = new Pose2d(2, 2, Rotation2d.kCCW_90deg);
    var routine = routine();
    pose = new Pose2d(2, 4, Rotation2d.kCCW_90deg);
    tick(0.02);
    tick(0.30);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.PASSED);
  }

  @Test
  void headingMustAlsoBeWithinTolerance() {
    var routine = routine();
    tick(0.02);
    pose = new Pose2d(4, 2, Rotation2d.fromDegrees(10));
    tick(1.0);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.DRIVING);
  }

  @BeforeEach
  void initialize() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DogLog.setOptions(
        new DogLogOptions().withLogExtras(false).withUseLogThread(false).withNtPublish(false));
    SimHooks.pauseTiming();
    trailblazer =
        new Trailblazer(
            new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
            new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(15, 0, 0)));
  }

  @Test
  void invalidSensorDataFailsBeforeCommandingMotion() {
    pose = new Pose2d(Double.NaN, 2, Rotation2d.kZero);
    var routine = routine();
    tick(0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.FAILED);
    assertThat(trailblazer.getFieldRelativeSetpoint(Pose2d.kZero, speeds).vxMetersPerSecond)
        .isEqualTo(0);
  }

  @Test
  void lateralDriftFailsEvenIfEventuallyAtGoal() {
    var routine = routine();
    tick(0.02);
    pose = new Pose2d(3, 2.3, Rotation2d.kZero);
    tick(0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.FAILED);
    assertThat(routine.reason()).contains("cross-track");
  }

  @Test
  void mustRemainStoppedForTheEntireSettleWindow() {
    var routine = routine();
    tick(0.02);
    pose = new Pose2d(4, 2, Rotation2d.kZero);
    tick(0.02);
    tick(0.20);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.SETTLING);
    speeds = new ChassisSpeeds(0.2, 0, 0);
    tick(0.02);
    speeds = new ChassisSpeeds();
    tick(0.02);
    tick(0.20);
    assertThat(routine.finished()).isFalse();
    tick(0.10);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.PASSED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isEqualTo(0);
  }

  @Test
  void nonfiniteSpeedFailsBeforeCommandingMotion() {
    speeds = new ChassisSpeeds(Double.NaN, 0, 0);
    var routine = routine();
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.FAILED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, new ChassisSpeeds()).vxMetersPerSecond)
        .isEqualTo(0);
  }

  @AfterEach
  void resumeClock() {
    SimHooks.resumeTiming();
  }

  @Test
  void rotationAlsoResetsTheSettleWindow() {
    var routine = routine();
    tick(0.02);
    pose = new Pose2d(4, 2, Rotation2d.kZero);
    tick(0.02);
    tick(0.20);
    speeds = new ChassisSpeeds(0, 0, Math.toRadians(10));
    tick(0.02);
    speeds = new ChassisSpeeds();
    tick(0.02);
    tick(0.20);
    assertThat(routine.finished()).isFalse();
    tick(0.10);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.PASSED);
  }

  @Test
  void stuckRobotFailsAndClearsMotion() {
    var routine = routine();
    tick(0.02);
    tick(10.1);
    assertThat(routine.getState()).isEqualTo(StraightLineTest.State.FAILED);
    assertThat(routine.reason()).contains("Timed out");
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isEqualTo(0);
  }
}
