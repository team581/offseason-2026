package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

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
import frc.robot.testing.StraightLineConfig;
import frc.robot.testing.StraightLineRoutine;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Deterministic failure-path checks, independent of asynchronous vendor physics. */
final class StraightLineRoutineTest {
  private static void tick(StraightLineRoutine routine, double seconds) {
    SimHooks.stepTimingAsync(seconds);
    routine.beforePeriodic();
    routine.periodic();
  }

  private Pose2d pose = new Pose2d(2, 2, Rotation2d.kZero);
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private boolean enabled;

  private Trailblazer trailblazer;

  private StraightLineRoutine routine() {
    return new StraightLineRoutine(
        StraightLineConfig.from(StraightLineConfig.DEFAULTS::get),
        trailblazer,
        () -> pose,
        () -> speeds,
        () -> enabled);
  }

  @Test
  void headingMustAlsoBeWithinTolerance() {
    var routine = routine();
    enabled = true;
    tick(routine, 0.02);
    pose = new Pose2d(4, 2, Rotation2d.fromDegrees(10));
    tick(routine, 1.0);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.DRIVING);
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
  void invalidConfigCannotStartATest() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                StraightLineConfig.from(
                    key ->
                        key.equals("distance")
                            ? Double.NaN
                            : StraightLineConfig.DEFAULTS.get(key)));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                StraightLineConfig.from(
                    key ->
                        key.equals("maxAcceleration") ? 0 : StraightLineConfig.DEFAULTS.get(key)));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                StraightLineConfig.from(
                    key ->
                        key.equals("positionTolerance")
                            ? 3
                            : StraightLineConfig.DEFAULTS.get(key)));
  }

  @Test
  void invalidSensorDataFailsBeforeCommandingMotion() {
    var routine = routine();
    pose = new Pose2d(Double.NaN, 2, Rotation2d.kZero);
    enabled = true;
    tick(routine, 0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.FAILED);
    assertThat(trailblazer.getFieldRelativeSetpoint(Pose2d.kZero, speeds).vxMetersPerSecond)
        .isZero();
  }

  @Test
  void lateralDriftFailsEvenIfEventuallyAtGoal() {
    var routine = routine();
    enabled = true;
    tick(routine, 0.02);
    pose = new Pose2d(3, 2.3, Rotation2d.kZero);
    tick(routine, 0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.FAILED);
    assertThat(routine.reason()).contains("cross-track");
  }

  @Test
  void mustRemainStoppedForTheEntireSettleWindow() {
    var routine = routine();
    enabled = true;
    tick(routine, 0.02);
    pose = new Pose2d(4, 2, Rotation2d.kZero);
    tick(routine, 0.02);
    tick(routine, 0.20);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.SETTLING);
    speeds = new ChassisSpeeds(0.2, 0, 0);
    tick(routine, 0.02);
    speeds = new ChassisSpeeds();
    tick(routine, 0.02);
    tick(routine, 0.20);
    assertThat(routine.finished()).isFalse();
    tick(routine, 0.10);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.PASSED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isZero();
  }

  @Test
  void negativeDistanceFollowsTheStartingRobotHeading() {
    pose = new Pose2d(2, 2, Rotation2d.kCCW_90deg);
    var config =
        StraightLineConfig.from(
            key -> key.equals("distance") ? -2 : StraightLineConfig.DEFAULTS.get(key));
    var routine =
        new StraightLineRoutine(config, trailblazer, () -> pose, () -> speeds, () -> enabled);
    enabled = true;
    tick(routine, 0.02);
    pose = new Pose2d(2, 0, Rotation2d.kCCW_90deg);
    tick(routine, 0.02);
    tick(routine, 0.3);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.PASSED);
  }

  @AfterEach
  void resumeClock() {
    SimHooks.resumeTiming();
  }

  @Test
  void stuckRobotFailsAndClearsMotion() {
    var routine = routine();
    enabled = true;
    tick(routine, 0.02);
    tick(routine, 10.1);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.FAILED);
    assertThat(routine.reason()).contains("Timed out");
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isZero();
  }

  @Test
  void waitsForEnableAndClearsMotionWhenInterrupted() {
    var routine = routine();
    tick(routine, 20);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.WAITING);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isZero();
    enabled = true;
    tick(routine, 0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.DRIVING);
    enabled = false;
    tick(routine, 0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.ABORTED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, speeds).vxMetersPerSecond).isZero();
  }
}
