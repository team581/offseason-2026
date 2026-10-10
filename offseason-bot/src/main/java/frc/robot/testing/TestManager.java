package frc.robot.testing;

import static com.google.common.collect.ImmutableMap.toImmutableMap;

import com.team581.math.PoseErrorTolerance;
import com.team581.swerve.TrailblazerDriveSource;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Runs a selected diagnostic once per Test-mode enable, independently of autonomous. */
public final class TestManager extends StateMachineSubsystem<TestManager.State> {
  public enum Selection {
    NONE,
    STRAIGHT_LINE
  }

  public enum State {
    INACTIVE,
    RUNNING,
    FINISHED,
    INVALID_CONFIG
  }

  private static final Map<String, DoubleSubscriber> TUNABLES =
      StraightLineConfig.DEFAULTS.entrySet().stream()
          .collect(
              toImmutableMap(
                  Map.Entry::getKey,
                  entry ->
                      DogLog.tunable(
                          "Tests/StraightLine/Config/" + entry.getKey(), entry.getValue())));

  private static Supplier<Selection> dashboardSelection() {
    var chooser = new SendableChooser<Selection>();
    chooser.setDefaultOption("NONE", Selection.NONE);
    chooser.addOption("STRAIGHT_LINE", Selection.STRAIGHT_LINE);
    SmartDashboard.putData("Tests/SelectedTest", chooser);
    return () -> Optional.ofNullable(chooser.getSelected()).orElse(Selection.NONE);
  }

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisSpeeds> speeds;
  private final Consumer<ChassisSpeeds> driveOutput;
  private final Runnable prepareForTest;
  private final Supplier<Selection> selectedTest;
  private final Supplier<StraightLineConfig> config;
  // A dedicated test controller. The production auto controller is left unchanged.
  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(
              new PIDController(3.5, 0, 0),
              new PIDController(
                  Swerve.ORIGINAL_HEADING_PID.getP(),
                  Swerve.ORIGINAL_HEADING_PID.getI(),
                  Swerve.ORIGINAL_HEADING_PID.getD())));
  private final TrailblazerDriveSource driveSource;
  private Optional<StraightLineRoutine> routine = Optional.empty();
  private Selection selection = Selection.NONE;
  private boolean testEnabled;
  private boolean previouslyEnabled;
  private boolean enteringTest;

  private boolean leavingTest;

  public TestManager(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds,
      Consumer<ChassisSpeeds> driveOutput,
      Runnable prepareForTest) {
    this(
        pose,
        speeds,
        driveOutput,
        prepareForTest,
        dashboardSelection(),
        () -> StraightLineConfig.from(key -> TUNABLES.get(key).get()));
  }

  /** Inject sensors, outputs, selection and configuration for the headless runner. */
  public TestManager(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds,
      Consumer<ChassisSpeeds> driveOutput,
      Runnable prepareForTest,
      Supplier<Selection> selectedTest,
      Supplier<StraightLineConfig> config) {
    super(SubsystemPriority.TEST_MANAGER, State.INACTIVE);
    this.pose = pose;
    this.speeds = speeds;
    this.driveOutput = driveOutput;
    this.prepareForTest = prepareForTest;
    this.selectedTest = selectedTest;
    this.config = config;
    driveSource = new TrailblazerDriveSource(trailblazer, pose, speeds);
  }

  public Optional<StraightLineRoutine> getRoutine() {
    return routine;
  }

  @Override
  protected void collectInputs() {
    testEnabled = DriverStation.isTestEnabled();
    enteringTest = testEnabled && !previouslyEnabled;
    leavingTest = !testEnabled && previouslyEnabled;
    previouslyEnabled = testEnabled;
  }

  @Override
  protected State getNextState(State current) {
    if (!testEnabled) {
      if (leavingTest) {
        routine.ifPresent(
            active -> {
              active.beforePeriodic();
              active.periodic();
            });
        trailblazer.clearActiveSegment();
        driveOutput.accept(new ChassisSpeeds());
      }
      return State.INACTIVE;
    }
    if (enteringTest) {
      selection = selectedTest.get();
      routine = Optional.empty();
      trailblazer.clearActiveSegment();
      if (selection == Selection.NONE) {
        return State.INACTIVE;
      }
      try {
        routine =
            Optional.of(
                new StraightLineRoutine(
                    config.get(), trailblazer, pose, speeds, DriverStation::isTestEnabled));
        DogLog.log("Tests/StraightLine/InvalidConfig", false);
        return State.RUNNING;
      } catch (IllegalArgumentException exception) {
        DogLog.log("Tests/StraightLine/InvalidConfig", true);
        DogLog.log("Tests/StraightLine/State", "INVALID_CONFIG");
        DogLog.log("Tests/StraightLine/Passed", false);
        DogLog.log("Tests/StraightLine/Reason", exception.getMessage());
        return State.INVALID_CONFIG;
      }
    }
    if (current == State.RUNNING && routine.orElseThrow().finished()) {
      return State.FINISHED;
    }
    return current;
  }

  @Override
  protected void whileInState(State state) {
    if (testEnabled) {
      prepareForTest.run();
      if (state == State.RUNNING) {
        var active = routine.orElseThrow();
        active.beforePeriodic();
        active.periodic();
        driveOutput.accept(driveSource.getRequestedSpeeds());
      } else {
        driveOutput.accept(new ChassisSpeeds());
      }
    }
    DogLog.log("Tests/ManagerState", state);
    DogLog.log("Tests/SelectedTest", selection);
  }
}
