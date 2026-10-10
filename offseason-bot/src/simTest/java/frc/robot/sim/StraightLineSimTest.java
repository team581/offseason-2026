package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.testing.StraightLineConfig;
import frc.robot.testing.StraightLineRoutine;
import frc.robot.testing.TestManager;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

final class StraightLineSimTest {
  private static void writeReport(
      StraightLineConfig config,
      double voltage,
      double heading,
      StraightLineRoutine routine,
      List<StraightLineRoutine.Sample> samples)
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

						Acceleration is the finite difference of measured velocity and includes vendor timing noise.
						""",
            routine.getState(),
            routine.reason(),
            config,
            voltage,
            heading,
            routine.sample().time(),
            routine.peakVelocity(),
            routine.peakAcceleration(),
            routine.maxCrossTrackError(),
            routine.sample().positionError()));
  }

  @Test
  @Timeout(120)
  void reachesGoalAndSettles() throws IOException {
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
    var config =
        StraightLineConfig.from(
            key ->
                Double.parseDouble(
                    System.getProperty(
                        "sim." + key, StraightLineConfig.DEFAULTS.get(key).toString())));
    double voltage = Double.parseDouble(System.getProperty("sim.batteryVoltage", "12.0"));
    double heading = Double.parseDouble(System.getProperty("sim.startHeading", "0.0"));
    if (!Double.isFinite(voltage) || voltage <= 0 || !Double.isFinite(heading)) {
      throw new IllegalArgumentException(
          "Battery voltage must be positive and start heading finite");
    }
    List<StraightLineRoutine.Sample> samples = new ArrayList<>();
    try (var fixture =
        new SwerveFixture(voltage, new Pose2d(2, 2, Rotation2d.fromDegrees(heading)))) {
      var manager =
          new TestManager(
              fixture::pose,
              fixture::speeds,
              fixture::applyRequest,
              () -> {},
              () -> TestManager.Selection.STRAIGHT_LINE,
              () -> config);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      try {
        HeadlessRunner.run(
            manager,
            () -> manager.getState() == TestManager.State.FINISHED,
            () -> {
              manager.getRoutine().ifPresent(active -> samples.add(active.sample()));
            },
            config.timeout() + 2.0);
      } finally {
        if (manager.getRoutine().isPresent()) {
          writeReport(config, voltage, heading, manager.getRoutine().orElseThrow(), samples);
        }
      }
      var routine = manager.getRoutine().orElseThrow();
      assertThat(routine.getState())
          .as(routine.reason())
          .isEqualTo(StraightLineRoutine.State.PASSED);
    }
  }
}
