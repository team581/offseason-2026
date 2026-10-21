package frc.robot.testing;

import static com.google.common.base.Preconditions.checkArgument;
import static org.assertj.core.api.Assertions.assertThat;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.Trailblazer;
import com.team581.trailblazer.followers.PidPathFollower;
import com.team581.trailblazer.trackers.HeuristicPathTracker;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.HeadlessRunner;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class HeadlessTest {
  private record DriveSample(
      double time, double commandedVx, double commandedVy, double measuredVx, double measuredVy) {
    static DriveSample of(double time, ChassisSpeeds commanded, ChassisSpeeds measured) {
      return new DriveSample(
          time,
          commanded.vxMetersPerSecond,
          commanded.vyMetersPerSecond,
          measured.vxMetersPerSecond,
          measured.vyMetersPerSecond);
    }
  }

  private static HeadlessRunner robot;

  private static void writeDriveSamples(List<DriveSample> samples) throws IOException {
    Path output =
        Path.of(System.getProperty("sim.outputDir", "build/reports/straightLineTest/motion"));
    Files.createDirectories(output);
    var csv =
        new StringBuilder(
            "time_s,commanded_vx_mps,commanded_vy_mps,measured_vx_mps,measured_vy_mps\n");
    double start = samples.get(0).time();
    for (var sample : samples) {
      csv.append(
          String.format(
              Locale.ROOT,
              "%.9f,%.9f,%.9f,%.9f,%.9f%n",
              sample.time() - start,
              sample.commandedVx(),
              sample.commandedVy(),
              sample.measuredVx(),
              sample.measuredVy()));
    }
    Files.writeString(output.resolve("drive-output.csv"), csv);
  }

  private static void writeReport(
      double voltage,
      double heading,
      StraightLineTest routine,
      List<StraightLineTest.Sample> samples)
      throws IOException {
    Path output =
        Path.of(System.getProperty("sim.outputDir", "build/reports/straightLineTest/motion"));
    Files.createDirectories(output);
    var csv =
        new StringBuilder(
            "time_s,x_m,y_m,heading_deg,vx_mps,vy_mps,velocity_mps,acceleration_mps2,cross_track_m,position_error_m\n");
    for (var sample : samples) {
      csv.append(
          String.format(
              Locale.ROOT,
              "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n",
              sample.time(),
              sample.pose().getX(),
              sample.pose().getY(),
              sample.pose().getRotation().getDegrees(),
              sample.vx(),
              sample.vy(),
              sample.velocity(),
              sample.acceleration(),
              sample.crossTrackError(),
              sample.positionError()));
    }
    Files.writeString(output.resolve("straight-line.csv"), csv);
    Files.writeString(
        output.resolve("summary.md"),
        String.format(
            Locale.ROOT,
            """
								# Straight-line test

								Result: **%s** — %s

								Configuration: `%s`

								Battery: %.2f V; initial heading: %.2f degrees

								| Measurement | Value |
								|---|---:|
								| Duration (s) | %.3f |
								| Peak speed (m/s) | %.3f |
								| Peak absolute acceleration (m/s²) | %.3f |
								| Maximum cross-track error (m) | %.3f |
								| Final position error (m) | %.3f |

								Acceleration is calculated from changes in measured velocity and includes simulation timing noise.

								Passing requires reaching the goal and settling. Acceleration limits are not checked.
								`drive-output.csv` includes the initial stopped sample, then the applied module targets converted to chassis speeds and
								the measured field-relative velocity from the same robot loop.
								""",
            routine.getState(),
            routine.reason(),
            "distance="
                + StraightLineTest.DISTANCE
                + ", maxVelocity="
                + StraightLineTest.MAX_VELOCITY
                + ", maxAcceleration="
                + StraightLineTest.MAX_ACCELERATION,
            voltage,
            heading,
            routine.sample().time(),
            routine.peakVelocity(),
            routine.peakAcceleration(),
            routine.maxCrossTrackError(),
            routine.sample().positionError()));
  }

  @AfterAll
  static void closeRobot() {
    if (robot != null) {
      robot.close();
    }
  }

  private Pose2d pose = new Pose2d(2, 2, Rotation2d.kZero);
  private ChassisSpeeds speeds = new ChassisSpeeds();

  private TestManager manager;
  private Trailblazer trailblazer;

  private StraightLineTest routine() {
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(true);
    tick(0.02);
    return (StraightLineTest) manager.getActiveTest().orElseThrow();
  }

  private void tick(double seconds) {
    DriverStationSim.notifyNewData();
    SimHooks.stepTimingAsync(seconds);
    manager.beforePeriodic();
    manager.periodic();
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
  void initialize(TestInfo info) {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DogLog.setOptions(
        new DogLogOptions().withLogExtras(false).withUseLogThread(false).withNtPublish(false));
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    if (info.getTags().contains("headless")) {
      return;
    }
    SimHooks.pauseTiming();
    trailblazer =
        new Trailblazer(
            new HeuristicPathTracker(new PoseErrorTolerance(0.5, 10)),
            new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(15, 0, 0)));
    manager =
        new TestManager(
            trailblazer,
            () -> pose,
            () -> speeds,
            () -> {},
            ignored -> {},
            () -> TestSelection.STRAIGHT_LINE);
    tick(0.02);
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
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
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

  @EnumSource(value = TestSelection.class, names = "NONE", mode = EnumSource.Mode.EXCLUDE)
  @ParameterizedTest
  @Tag("headless")
  @Timeout(120)
  void runsRegisteredTest(TestSelection selection) throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", selection.name()).equals(selection.name()));
    SimHooks.resumeTiming();
    assertThat(HAL.initialize(500, 0)).as("Initialize desktop HAL").isTrue();
    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withUseLogThread(false)
            .withNtPublish(false)
            .withNtTunables(false));
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    double voltage = Double.parseDouble(System.getProperty("sim.batteryVoltage", "12.0"));
    double heading = Double.parseDouble(System.getProperty("sim.startHeading", "0.0"));
    checkArgument(
        Double.isFinite(voltage) && voltage > 0 && Double.isFinite(heading),
        "Battery voltage must be positive and start heading finite");
    List<StraightLineTest.Sample> samples = new ArrayList<>();
    List<DriveSample> driveSamples = new ArrayList<>();
    RoboRioSim.setVInVoltage(voltage);
    if (robot == null) {
      robot = new HeadlessRunner();
    }
    // Avoid the auto chooser resetting the test's starting pose while disabled.
    try (var resetPose =
        NetworkTableInstance.getDefault()
            .getTable("DSOptions")
            .getBooleanTopic("ResetPoseForAuto")
            .publish()) {
      resetPose.set(false);
      robot.select(selection, new Pose2d(2, 2, Rotation2d.fromDegrees(heading)));
      var manager = robot.manager();
      String previousRun = manager.getRunId();
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      // Record the stopped speed before the first drive command.
      driveSamples.add(
          DriveSample.of(Timer.getFPGATimestamp(), new ChassisSpeeds(), robot.measuredSpeeds()));
      try {
        robot.runUntil(
            () ->
                !manager.getRunId().equals(previousRun)
                    && manager.getResult().filter(TestManager.Result::finished).isPresent(),
            () -> {
              var requested = robot.commandedSpeeds();
              driveSamples.add(
                  DriveSample.of(Timer.getFPGATimestamp(), requested, robot.measuredSpeeds()));
              manager
                  .getActiveTest()
                  .filter(StraightLineTest.class::isInstance)
                  .map(StraightLineTest.class::cast)
                  .ifPresent(active -> samples.add(active.sample()));
            },
            30.0);
      } finally {
        DriverStationSim.setEnabled(false);
        robot.tick();
        writeDriveSamples(driveSamples);
        if (manager.getActiveTest().orElse(null) instanceof StraightLineTest routine) {
          writeReport(voltage, heading, routine, samples);
        }
        var result = manager.getResult().orElseThrow();
        Path output =
            Path.of(System.getProperty("sim.outputDir", "build/reports/headlessTest/motion"));
        Files.writeString(
            output.resolve(selection.name().toLowerCase(Locale.ROOT) + ".md"),
            "# " + selection + "\n\n" + result.status() + ": " + result.reason() + "\n");
      }
      var result = manager.getResult().orElseThrow();
      assertThat(result.status()).as(result.reason()).isEqualTo(TestManager.Status.PASSED);
    }
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
