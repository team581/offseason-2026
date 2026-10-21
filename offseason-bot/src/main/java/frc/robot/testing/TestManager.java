package frc.robot.testing;

import static java.util.Objects.requireNonNullElse;
import static java.util.UUID.randomUUID;

import com.team581.swerve.DriveSourceType;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.state_machines.StateMachine;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Selects tests like Autos and runs the same state machines on the robot or in headless sim. */
public final class TestManager extends StateMachineSubsystem<TestSelection> {
  public record Result(String name, Status status, String reason) {
    public boolean finished() {
      return status != Status.RUNNING;
    }
  }

  public enum Status {
    RUNNING,
    PASSED,
    FAILED,
    ABORTED
  }

  public abstract static class Test<S extends Enum<S>> extends StateMachine<S> {
    protected Test(S initialState) {
      super(initialState);
    }

    public abstract void abort();

    public abstract Result result();
  }

  private static Supplier<TestSelection> dashboardSelection() {
    var chooser = new SendableChooser<TestSelection>();
    for (var selection : TestSelection.values()) {
      chooser.addOption(selection.toString(), selection);
    }
    chooser.setDefaultOption(TestSelection.NONE.toString(), TestSelection.NONE);
    SmartDashboard.putData("Tests/SelectedTest", chooser);
    return () -> requireNonNullElse(chooser.getSelected(), TestSelection.NONE);
  }

  final Trailblazer trailblazer;
  Pose2d pose = Pose2d.kZero;
  ChassisSpeeds speeds = new ChassisSpeeds();
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final Runnable prepare;
  private final Consumer<DriveSourceType> driveSource;
  private final Supplier<TestSelection> selection;
  private Optional<Test<?>> active = Optional.empty();
  private Optional<Result> result = Optional.empty();
  private boolean previouslyEnabled;
  private boolean startRequested;
  private String runId = "NOT_STARTED";

  public TestManager(RobotManager robotManager, Trailblazer trailblazer) {
    this(
        trailblazer,
        robotManager.localization::getPose,
        robotManager.swerve::getFieldRelativeSpeeds,
        () -> {
          robotManager.idleRequest();
          robotManager.cancelIntakeRequest();
          robotManager.hopperManager.setDriverWantsEject(false);
        },
        robotManager.swerve::setDriveSourceType,
        dashboardSelection());
  }

  public TestManager(
      Trailblazer trailblazer,
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds,
      Runnable prepare,
      Consumer<DriveSourceType> driveSource,
      Supplier<TestSelection> selection) {
    super(SubsystemPriority.TEST_MANAGER, TestSelection.NONE);
    this.trailblazer = trailblazer;
    poseSupplier = pose;
    speedsSupplier = speeds;
    this.prepare = prepare;
    this.driveSource = driveSource;
    this.selection = selection;
  }

  public Optional<Test<?>> getActiveTest() {
    return active;
  }

  public Optional<Result> getResult() {
    return result;
  }

  public String getRunId() {
    return runId;
  }

  /** Starts another run of the selected test, including without leaving enabled Test mode. */
  public void runRequest() {
    startRequested = true;
  }

  private void abort() {
    try {
      if (active.isPresent() && !active.orElseThrow().result().finished()) {
        active.orElseThrow().abort();
        result = Optional.of(active.orElseThrow().result());
      }
    } finally {
      trailblazer.clearActiveSegment();
    }
  }

  @Override
  protected void collectInputs() {
    pose = poseSupplier.get();
    speeds = speedsSupplier.get();
  }

  @Override
  protected TestSelection getNextState(TestSelection current) {
    return DriverStation.isEnabled() ? current : selection.get();
  }

  @Override
  protected void whileInState(TestSelection selected) {
    boolean enabled = DriverStation.isTestEnabled();
    try {
      if (enabled) {
        prepare.run();
        driveSource.accept(DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP);
        if (!previouslyEnabled || startRequested) {
          abort();
          active = Optional.empty();
          result = Optional.empty();
          runId = randomUUID().toString();
          active = selected.test.apply(this);
        }
        if (active.isPresent() && !active.orElseThrow().result().finished()) {
          var test = active.orElseThrow();
          test.beforePeriodic();
          test.periodic();
          result = Optional.of(test.result());
          if (test.result().finished()) {
            trailblazer.clearActiveSegment();
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
      active = Optional.empty();
      result = Optional.of(new Result(selected.name(), Status.FAILED, exception.toString()));
    } finally {
      previouslyEnabled = enabled;
      startRequested = false;
    }
    DogLog.log("Tests/SelectedTest", selected.name());
    DogLog.log("Tests/Regression/RunId", runId);
    DogLog.log("Tests/Regression/Status", result.map(r -> r.status().name()).orElse("INACTIVE"));
    DogLog.log(
        "Tests/Regression/Passed", result.filter(r -> r.status() == Status.PASSED).isPresent());
    DogLog.log("Tests/Reason", result.map(Result::reason).orElse("No test selected"));
  }
}
