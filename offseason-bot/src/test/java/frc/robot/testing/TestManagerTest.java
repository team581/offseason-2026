package frc.robot.testing;

import static com.google.common.base.Preconditions.checkState;
import static org.assertj.core.api.Assertions.assertThat;

import com.team581.math.PoseErrorTolerance;
import com.team581.swerve.DriveSourceType;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class TestManagerTest {
  private Pose2d pose = Pose2d.kZero;
  private TestSelection selection = TestSelection.STRAIGHT_LINE;
  private DriveSourceType source = DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;
  private Trailblazer trailblazer;
  private TestManager manager;
  private int prepareCalls;
  private boolean failPreparation;

  private void enable() {
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(true);
    tick(0.02);
  }

  private void tick(double seconds) {
    DriverStationSim.notifyNewData();
    SimHooks.stepTimingAsync(seconds);
    manager.beforePeriodic();
    manager.periodic();
  }

  @AfterEach
  void cleanup() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @Test
  void completedTestCanRunAgainWithoutDisabling() {
    enable();
    var first = manager.getActiveTest().orElseThrow();
    var firstRun = manager.getRunId();
    pose = new Pose2d(2, 0, pose.getRotation());
    tick(0.02);
    tick(0.30);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.PASSED);
    tick(1.0);
    assertThat(manager.getActiveTest()).containsSame(first);
    manager.runRequest();
    tick(0.02);
    assertThat(manager.getActiveTest().orElseThrow()).isNotSameAs(first);
    assertThat(manager.getRunId()).isNotEqualTo(firstRun);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.RUNNING);
    pose = new Pose2d(4, 0, pose.getRotation());
    tick(0.02);
    tick(0.30);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.PASSED);
  }

  @Test
  void disablingAbortsAndClearsTrailblazer() {
    enable();
    DriverStationSim.setEnabled(false);
    tick(0.02);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.ABORTED);
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, new ChassisSpeeds()).vxMetersPerSecond)
        .isEqualTo(0);
    assertThat(source).isEqualTo(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
  }

  @Test
  void exceptionsStopMotionAndTheNextEnableCanRetry() {
    enable();
    failPreparation = true;
    tick(0.02);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.FAILED);
    assertThat(manager.getResult().orElseThrow().reason()).contains("Preparation failed");
    assertThat(trailblazer.getFieldRelativeSetpoint(pose, new ChassisSpeeds()).vxMetersPerSecond)
        .isEqualTo(0);
    failPreparation = false;
    tick(0.02);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.FAILED);
    DriverStationSim.setEnabled(false);
    tick(0.02);
    enable();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.RUNNING);
  }

  @BeforeEach
  void initialize() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DogLog.setOptions(
        new DogLogOptions().withLogExtras(false).withUseLogThread(false).withNtPublish(false));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    SimHooks.pauseTiming();
    trailblazer =
        new Trailblazer(
            new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
            new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(15, 0, 0)));
    manager =
        new TestManager(
            trailblazer,
            () -> pose,
            ChassisSpeeds::new,
            () -> {
              prepareCalls++;
              checkState(!failPreparation, "Preparation failed");
            },
            value -> source = value,
            () -> selection);
    tick(0.02);
  }

  @Test
  void onlyRunsInEnabledTestMode() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    tick(0.02);
    DriverStationSim.setAutonomous(false);
    tick(0.02);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(false);
    tick(0.02);
    assertThat(manager.getActiveTest()).isEmpty();
    assertThat(prepareCalls).isEqualTo(0);
    enable();
    assertThat(source).isEqualTo(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestManager.Status.RUNNING);
  }

  @Test
  void selectionMatchesAutosAndEveryEnableCreatesAFreshTest() {
    enable();
    var first = manager.getActiveTest().orElseThrow();
    selection = TestSelection.NONE;
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestSelection.STRAIGHT_LINE);
    DriverStationSim.setEnabled(false);
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestSelection.NONE);
    enable();
    assertThat(manager.getActiveTest()).isEmpty();
    DriverStationSim.setEnabled(false);
    selection = TestSelection.STRAIGHT_LINE;
    tick(0.02);
    enable();
    assertThat(manager.getActiveTest().orElseThrow()).isNotSameAs(first);
  }
}
