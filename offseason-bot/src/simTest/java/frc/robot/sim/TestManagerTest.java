package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.testing.StraightLineConfig;
import frc.robot.testing.StraightLineRoutine;
import frc.robot.testing.TestManager;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class TestManagerTest {
  private Pose2d pose = new Pose2d(2, 2, Rotation2d.kZero);
  private ChassisSpeeds measured = new ChassisSpeeds();
  private ChassisSpeeds output = new ChassisSpeeds();
  private int outputCalls;
  private int prepareCalls;
  private double distance = 2;
  private TestManager.Selection selection = TestManager.Selection.STRAIGHT_LINE;
  private TestManager manager;

  private void enableTest() {
    DriverStationSim.setAutonomous(false);
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

  @Test
  void cannotRunInAutoTeleopOrDisabledTestMode() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    tick(0.02);
    DriverStationSim.setAutonomous(false);
    tick(0.02);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(false);
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestManager.State.INACTIVE);
    assertThat(manager.getRoutine()).isEmpty();
    assertThat(outputCalls).isZero();
    assertThat(prepareCalls).isZero();
  }

  @AfterEach
  void cleanup() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  @BeforeEach
  void initialize() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DogLog.setOptions(
        new DogLogOptions().withLogExtras(false).withUseLogThread(false).withNtPublish(false));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    SimHooks.pauseTiming();
    manager =
        new TestManager(
            () -> pose,
            () -> measured,
            requested -> {
              output = requested;
              outputCalls++;
            },
            () -> prepareCalls++,
            () -> selection,
            () ->
                StraightLineConfig.from(
                    key ->
                        key.equals("distance") ? distance : StraightLineConfig.DEFAULTS.get(key)));
  }

  @Test
  void invalidConfigStaysStoppedUntilNextEnable() {
    distance = 0;
    enableTest();
    assertThat(manager.getState()).isEqualTo(TestManager.State.INVALID_CONFIG);
    assertThat(manager.getRoutine()).isEmpty();
    assertThat(output.vxMetersPerSecond).isZero();
    distance = 2;
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestManager.State.INVALID_CONFIG);
    DriverStationSim.setEnabled(false);
    tick(0.02);
    enableTest();
    assertThat(manager.getState()).isEqualTo(TestManager.State.RUNNING);
  }

  @Test
  void leavingTestModeAbortsAndClearsItsOutput() {
    enableTest();
    var routine = manager.getRoutine().orElseThrow();
    DriverStationSim.setTest(false);
    DriverStationSim.setAutonomous(true);
    tick(0.02);
    assertThat(routine.getState()).isEqualTo(StraightLineRoutine.State.ABORTED);
    assertThat(manager.getState()).isEqualTo(TestManager.State.INACTIVE);
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(output.vyMetersPerSecond).isZero();
    assertThat(output.omegaRadiansPerSecond).isZero();
    tick(0.02);
    assertThat(manager.getRoutine().orElseThrow()).isSameAs(routine);
  }

  @Test
  void noneSelectionHoldsZeroAndChangingSelectionRequiresReenable() {
    selection = TestManager.Selection.NONE;
    enableTest();
    selection = TestManager.Selection.STRAIGHT_LINE;
    tick(0.02);
    assertThat(manager.getRoutine()).isEmpty();
    assertThat(output.vxMetersPerSecond).isZero();
    DriverStationSim.setEnabled(false);
    tick(0.02);
    enableTest();
    assertThat(manager.getState()).isEqualTo(TestManager.State.RUNNING);
    assertThat(manager.getRoutine()).isPresent();
    assertThat(prepareCalls).isPositive();
  }

  @Test
  void selectionAndConfigurationAreFixedUntilNextTestEnable() {
    enableTest();
    var first = manager.getRoutine().orElseThrow();
    distance = 1;
    selection = TestManager.Selection.NONE;
    pose = new Pose2d(4, 2, Rotation2d.kZero);
    tick(0.02);
    tick(0.30);
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestManager.State.FINISHED);
    assertThat(first.getState()).isEqualTo(StraightLineRoutine.State.PASSED);
    assertThat(output.vxMetersPerSecond).isZero();
    tick(1.0);
    assertThat(manager.getRoutine().orElseThrow()).isSameAs(first);
    selection = TestManager.Selection.STRAIGHT_LINE;
    DriverStationSim.setEnabled(false);
    tick(0.02);
    enableTest();
    assertThat(manager.getRoutine().orElseThrow()).isNotSameAs(first);
    assertThat(manager.getRoutine().orElseThrow().sample().positionError()).isEqualTo(1);
  }

  @Test
  void strictRegressionRejectsMovingStartAndStopsDependents() {
    selection = TestManager.Selection.DRIVE_REGRESSION;
    measured = new ChassisSpeeds(0.2, 0, 0);
    enableTest();
    tick(0.02);
    assertThat(manager.getState()).isEqualTo(TestManager.State.FINISHED);
    assertThat(manager.getResults())
        .extracting(frc.robot.testing.DiagnosticRoutine.Result::status)
        .containsExactly(
            frc.robot.testing.DiagnosticRoutine.Status.FAILED,
                frc.robot.testing.DiagnosticRoutine.Status.BLOCKED,
            frc.robot.testing.DiagnosticRoutine.Status.BLOCKED,
                frc.robot.testing.DiagnosticRoutine.Status.BLOCKED);
    assertThat(manager.getResults().get(0).reason()).contains("stopped");
    assertThat(output.vxMetersPerSecond).isZero();
    assertThat(output.vyMetersPerSecond).isZero();
    assertThat(output.omegaRadiansPerSecond).isZero();
  }
}
