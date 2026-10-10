package frc.robot.testing;

import static com.google.common.collect.ImmutableMap.toImmutableMap;
import static java.util.Objects.requireNonNullElse;

import com.team581.math.PoseErrorTolerance;
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
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Test-mode adapter. Its lifecycle/controllers are separate from production robot management. */
public final class TestManager extends StateMachineSubsystem<TestManager.State> {
  public enum Selection {
    NONE,
    STRAIGHT_LINE,
    DRIVE_REGRESSION
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
    chooser.setDefaultOption(Selection.NONE.name(), Selection.NONE);
    for (var option : Selection.values()) {
      if (option != Selection.NONE) {
        chooser.addOption(option.name(), option);
      }
    }
    SmartDashboard.putData("Tests/SelectedTest", chooser);
    return () -> requireNonNullElse(chooser.getSelected(), Selection.NONE);
  }

  // Dedicated diagnostic controller; does not share controller state with autonomous.
  private final Trailblazer trailblazer =
      new Trailblazer(
          new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
          new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(15, 0, 0)));
  private final DiagnosticSession session;
  private Selection selection = Selection.NONE;
  private String runId = "NOT_STARTED";

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

  // Test-owned injection points; no production manager implements a diagnostic interface.
  public TestManager(
      Supplier<Pose2d> pose,
      Supplier<ChassisSpeeds> speeds,
      Consumer<ChassisSpeeds> driveOutput,
      Runnable prepareForTest,
      Supplier<Selection> selectedTest,
      Supplier<StraightLineConfig> config) {
    super(SubsystemPriority.TEST_MANAGER, State.INACTIVE);
    session =
        new DiagnosticSession(
            () -> {
              runId = java.util.UUID.randomUUID().toString();
              selection = requireNonNullElse(selectedTest.get(), Selection.NONE);
              trailblazer.clearActiveSegment();
              if (selection == Selection.NONE) {
                return Optional.empty();
              }
              var snapshot = config.get();
              if (selection == Selection.STRAIGHT_LINE) {
                return Optional.of(
                    new StraightLineDiagnostic(
                        "StraightLine", snapshot, trailblazer, pose, speeds, false));
              }
              // Four independently judged legs form a square relative to the initial heading.
              var steps = new java.util.ArrayList<DiagnosticSequence.Step>();
              for (int i = 0; i < 4; i++) {
                double direction = snapshot.direction() + i * 90;
                var legConfig =
                    new StraightLineConfig(
                        snapshot.distance(),
                        direction,
                        snapshot.maxVelocity(),
                        snapshot.maxAcceleration(),
                        snapshot.timeout(),
                        snapshot.positionTolerance(),
                        snapshot.headingTolerance(),
                        snapshot.stoppedVelocity(),
                        snapshot.settleSeconds(),
                        snapshot.crossTrackTolerance());
                String name = "DriveLeg" + (i + 1);
                steps.add(
                    new DiagnosticSequence.Step(
                        name,
                        snapshot.timeout() + 0.02,
                        () ->
                            new StraightLineDiagnostic(
                                name, legConfig, trailblazer, pose, speeds, true)));
              }
              return Optional.of(new DiagnosticSequence(steps));
            },
            prepareForTest,
            driveOutput);
  }

  public Optional<DiagnosticRoutine> getActiveDiagnostic() {
    return session.active();
  }

  public List<DiagnosticRoutine.Result> getResults() {
    return session.results();
  }

  // Existing straight-line callers retain access to their motion samples.
  public Optional<StraightLineRoutine> getRoutine() {
    return session
        .active()
        .filter(StraightLineDiagnostic.class::isInstance)
        .map(StraightLineDiagnostic.class::cast)
        .map(StraightLineDiagnostic::routine);
  }

  public String getRunId() {
    return runId;
  }

  @Override
  protected State getNextState(State current) {
    session.tick(DriverStation.isTestEnabled());
    return State.valueOf(session.state().name());
  }

  @Override
  protected void whileInState(State state) {
    DogLog.log("Tests/ManagerState", state);
    DogLog.log("Tests/SelectedTest", selection);
    DogLog.log("Tests/Reason", session.reason());
    DogLog.log("Tests/Regression/RunId", runId);
    DogLog.log(
        "Tests/Regression/Status",
        session.result().map(result -> result.status().name()).orElse("INACTIVE"));
    DogLog.log(
        "Tests/Regression/Passed",
        session
            .result()
            .map(result -> result.status() == DiagnosticRoutine.Status.PASSED)
            .orElse(false));
    if (state == State.INVALID_CONFIG) {
      DogLog.log("Tests/StraightLine/InvalidConfig", true);
      DogLog.log("Tests/StraightLine/State", "INVALID_CONFIG");
      DogLog.log("Tests/StraightLine/Passed", false);
      DogLog.log("Tests/StraightLine/Reason", session.reason());
    } else if (state == State.RUNNING) {
      DogLog.log("Tests/StraightLine/InvalidConfig", false);
    }
    session
        .result()
        .ifPresent(
            overall -> {
              for (var result : getResults()) {
                DogLog.log(
                    "Tests/Regression/Steps/" + result.name() + "/Status", result.status().name());
                DogLog.log("Tests/Regression/Steps/" + result.name() + "/Reason", result.reason());
              }
            });
  }
}
