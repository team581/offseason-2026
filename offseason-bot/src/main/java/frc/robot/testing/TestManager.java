package frc.robot.testing;

import static java.util.Objects.requireNonNullElse;

import com.team581.swerve.DriveSourceType;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.DSOptions;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

/** Selects tests like Autos and runs the same state machines on the robot or in headless sim. */
public final class TestManager extends StateMachineSubsystem<TestSelection> {
  private static Supplier<TestSelection> dashboardSelection() {
    var chooser = new SendableChooser<TestSelection>();
    for (var selection : TestSelection.values()) {
      chooser.addOption(selection.toString(), selection);
    }
    chooser.setDefaultOption(TestSelection.NONE.toString(), TestSelection.NONE);
    SmartDashboard.putData("Tests/SelectedTest", chooser);
    return () -> requireNonNullElse(chooser.getSelected(), TestSelection.NONE);
  }

  private final Supplier<TestSelection> selection;
  private final Function<TestSelection, RobotTest<?>> createTest;
  private final Consumer<DriveSourceType> driveSource;
  private final Runnable stopMotion;
  private Optional<RobotTest<?>> active = Optional.empty();
  private Optional<TestResult> result = Optional.empty();
  private boolean previouslyEnabled;

  public TestManager(RobotManager robotManager, Trailblazer trailblazer) {
    this(
        dashboardSelection(),
        selected -> selected.create(robotManager, trailblazer),
        robotManager.swerve::setDriveSourceType,
        trailblazer::clearActiveSegment);
  }

  TestManager(
      Supplier<TestSelection> selection,
      Function<TestSelection, RobotTest<?>> createTest,
      Consumer<DriveSourceType> driveSource,
      Runnable stopMotion) {
    super(SubsystemPriority.TEST_MANAGER, TestSelection.NONE);
    this.selection = selection;
    this.createTest = createTest;
    this.driveSource = driveSource;
    this.stopMotion = stopMotion;
  }

  public Optional<RobotTest<?>> getActiveTest() {
    return active;
  }

  public Optional<TestResult> getResult() {
    return result;
  }

  public String getRunId() {
    return active.map(RobotTest::getRunId).orElse("NOT_STARTED");
  }

  /** Keeps drive control until TestMode is turned off, including after a test finishes. */
  public boolean isActive() {
    return previouslyEnabled;
  }

  private void abort() {
    try {
      if (active.isPresent() && !active.orElseThrow().result().finished()) {
        active.orElseThrow().abort();
        result = Optional.of(active.orElseThrow().result());
      }
    } finally {
      stopMotion.run();
    }
  }

  @Override
  protected TestSelection getNextState(TestSelection current) {
    return DriverStation.isEnabled() ? current : selection.get();
  }

  @Override
  protected void whileInState(TestSelection selected) {
    boolean enabled =
        DriverStation.isTeleopEnabled()
            && DSOptions.TEST_MODE.getAsBoolean()
            && selected != TestSelection.NONE;
    try {
      if (enabled) {
        driveSource.accept(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
        if (!previouslyEnabled) {
          active = Optional.empty();
          result = Optional.empty();
          active = Optional.of(createTest.apply(selected));
        }
        if (active.isPresent() && result.filter(TestResult::finished).isEmpty()) {
          var test = active.orElseThrow();
          test.beforePeriodic();
          test.periodic();
          result = Optional.of(test.result());
          if (test.result().finished()) {
            stopMotion.run();
          }
        }
      } else if (previouslyEnabled) {
        abort();
        driveSource.accept(DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP);
      }
    } catch (RuntimeException exception) {
      try {
        abort();
      } catch (RuntimeException cleanupException) {
        exception.addSuppressed(cleanupException);
      }
      result =
          Optional.of(new TestResult(selected.name(), TestStatus.FAILED, exception.toString()));
    } finally {
      previouslyEnabled = enabled;
    }
    DogLog.log("Tests/SelectedTest", selected.name());
    DogLog.log("Tests/Regression/RunId", getRunId());
    DogLog.log("Tests/Regression/Status", result.map(r -> r.status().name()).orElse("INACTIVE"));
    DogLog.log(
        "Tests/Regression/Passed", result.filter(r -> r.status() == TestStatus.PASSED).isPresent());
    DogLog.log("Tests/Reason", result.map(TestResult::reason).orElse("No test selected"));
  }
}
