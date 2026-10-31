package frc.robot.testing;

import static com.google.common.base.Preconditions.checkState;
import static org.assertj.core.api.Assertions.assertThat;

import com.team581.swerve.DriveSourceType;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class TestManagerTest {
  private static final class FakeTest extends RobotTest<TestStatus> {
    private int ticks;
    private boolean complete;
    private boolean fail;
    private boolean failAbort;

    FakeTest() {
      super(TestStatus.RUNNING);
    }

    @Override
    public void abort() {
      checkState(!failAbort, "Abort failed");
      setStateFromRequest(TestStatus.ABORTED);
    }

    @Override
    public TestResult result() {
      return new TestResult("FakeTest", getState(), "Test result");
    }

    @Override
    protected TestStatus getNextState(TestStatus current) {
      ticks++;
      checkState(!fail, "Test failed");
      return complete ? TestStatus.PASSED : current;
    }
  }

  private TestSelection selection = TestSelection.STRAIGHT_LINE;
  private DriveSourceType source = DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;
  private TestManager manager;
  private FakeTest test;
  private int created;
  private int driveChanges;
  private int stops;
  private boolean failCreation;

  private void enable() {
    setTestMode(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    tick();
  }

  private void setTestMode(boolean enabled) {
    NetworkTableInstance.getDefault()
        .getTable("DSOptions")
        .getEntry("TestMode")
        .setBoolean(enabled);
  }

  private void tick() {
    DriverStationSim.notifyNewData();
    SimHooks.stepTimingAsync(0.02);
    manager.beforePeriodic();
    manager.periodic();
  }

  @AfterEach
  void cleanup() {
    setTestMode(false);
    DriverStationSim.setEnabled(false);
    tick();
    SimHooks.resumeTiming();
  }

  @Test
  void completedResultStaysVisibleUntilTestModeIsToggled() {
    enable();
    var first = test;
    var firstRun = manager.getRunId();
    test.complete = true;
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.PASSED);
    assertThat(stops).isEqualTo(1);
    int completedTicks = test.ticks;
    tick();
    assertThat(test.ticks).isEqualTo(completedTicks);
    assertThat(manager.isActive()).isTrue();
    assertThat(source).isEqualTo(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
    setTestMode(false);
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.PASSED);
    setTestMode(true);
    tick();
    assertThat(test).isNotSameAs(first);
    assertThat(manager.getRunId()).isNotEqualTo(firstRun);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.RUNNING);
  }

  @Test
  void constructionFailuresAreReportedWithoutRetryingEachLoop() {
    failCreation = true;
    enable();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.FAILED);
    assertThat(manager.getResult().orElseThrow().reason()).contains("Creation failed");
    assertThat(stops).isEqualTo(1);
    tick();
    assertThat(created).isEqualTo(1);
    failCreation = false;
    setTestMode(false);
    tick();
    enable();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.RUNNING);
  }

  @Test
  void disablingAbortsAndStopsMotion() {
    enable();
    DriverStationSim.setEnabled(false);
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.ABORTED);
    assertThat(stops).isEqualTo(1);
    assertThat(manager.isActive()).isFalse();
    assertThat(source).isEqualTo(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
  }

  @Test
  void exceptionsStopMotionAndWaitForTheNextActivation() {
    enable();
    test.fail = true;
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.FAILED);
    assertThat(manager.getResult().orElseThrow().reason()).contains("Test failed");
    assertThat(stops).isEqualTo(1);
    int failedTicks = test.ticks;
    tick();
    assertThat(test.ticks).isEqualTo(failedTicks);
    DriverStationSim.setEnabled(false);
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.FAILED);
    enable();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.RUNNING);
  }

  @Test
  void failingAbortStillStopsMotion() {
    enable();
    test.failAbort = true;
    setTestMode(false);
    tick();
    assertThat(stops).isPositive();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.FAILED);
    assertThat(manager.getResult().orElseThrow().reason()).contains("Abort failed");
    assertThat(manager.isActive()).isFalse();
  }

  @BeforeEach
  void initialize() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DogLog.setOptions(
        new DogLogOptions().withLogExtras(false).withUseLogThread(false).withNtPublish(false));
    setTestMode(false);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    SimHooks.pauseTiming();
    manager =
        new TestManager(
            () -> selection,
            selected -> {
              created++;
              checkState(!failCreation, "Creation failed");
              test = new FakeTest();
              return test;
            },
            value -> {
              source = value;
              driveChanges++;
            },
            () -> stops++);
    tick();
  }

  @Test
  void noneDoesNotChangeDriveControlOrStartATest() {
    selection = TestSelection.NONE;
    tick();
    enable();
    tick();
    assertThat(manager.getActiveTest()).isEmpty();
    assertThat(manager.getResult()).isEmpty();
    assertThat(manager.isActive()).isFalse();
    assertThat(created).isEqualTo(0);
    assertThat(driveChanges).isEqualTo(0);
    assertThat(stops).isEqualTo(0);
  }

  @Test
  void onlyRunsInEnabledTeleopWithTestModeOption() {
    DriverStationSim.setEnabled(true);
    tick();
    setTestMode(true);
    DriverStationSim.setAutonomous(true);
    tick();
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    tick();
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    tick();
    assertThat(manager.isActive()).isFalse();
    assertThat(created).isEqualTo(0);
    DriverStationSim.setEnabled(true);
    tick();
    assertThat(source).isEqualTo(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.RUNNING);
  }

  @Test
  void selectionIsReadOnlyWhileDisabledAndEveryEnableCreatesAFreshTest() {
    enable();
    var first = test;
    selection = TestSelection.NONE;
    tick();
    assertThat(manager.getState()).isEqualTo(TestSelection.STRAIGHT_LINE);
    DriverStationSim.setEnabled(false);
    tick();
    assertThat(manager.getState()).isEqualTo(TestSelection.NONE);
    int previousDriveChanges = driveChanges;
    int previousStops = stops;
    enable();
    assertThat(manager.isActive()).isFalse();
    assertThat(created).isEqualTo(1);
    assertThat(driveChanges).isEqualTo(previousDriveChanges);
    assertThat(stops).isEqualTo(previousStops);
    DriverStationSim.setEnabled(false);
    selection = TestSelection.STRAIGHT_LINE;
    tick();
    enable();
    assertThat(test).isNotSameAs(first);
  }

  @Test
  void turningOffTestModeAbortsAndAllowsAFreshRunWhileEnabled() {
    enable();
    var first = test;
    setTestMode(false);
    tick();
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.ABORTED);
    assertThat(stops).isEqualTo(1);
    assertThat(source).isEqualTo(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
    setTestMode(true);
    tick();
    assertThat(test).isNotSameAs(first);
    assertThat(manager.getResult().orElseThrow().status()).isEqualTo(TestStatus.RUNNING);
  }
}
