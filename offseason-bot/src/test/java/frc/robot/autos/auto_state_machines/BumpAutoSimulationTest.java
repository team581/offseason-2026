package frc.robot.autos.auto_state_machines;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.mechanisms.imu.BumpCrossingState;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.HeadlessRunner;
import frc.robot.Robot;
import frc.robot.autos.Autos;
import frc.robot.autos.BaseImperativeAuto;
import java.util.HashSet;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

@Tag("headless")
final class BumpAutoSimulationTest {
  @Test
  @Timeout(150)
  void completesAllCyclesWithRealTurnsAndDetectedCrossings() throws Exception {
    HAL.initialize(500, 0);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(
        Boolean.getBoolean("sim.red") ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    try (var robot = new HeadlessRunner()) {
      var autoField = Robot.class.getDeclaredField("autos");
      autoField.setAccessible(true);
      var autos = (Autos) autoField.get(robot);
      var selectedField = Autos.class.getDeclaredField("selectedAuto");
      selectedField.setAccessible(true);
      String selection = System.getProperty("sim.auto", "LEFT_BUMP_CROSS");
      NetworkTableInstance.getDefault()
          .getTable("SmartDashboard")
          .getSubTable("Autos/SelectedAuto")
          .getEntry("selected")
          .setString(selection);
      robot.runUntil(
          () ->
              autos.getState().name().equals(selection)
                  && robot.robotManager.swerve.drivetrain.isOdometryValid(),
          5.0);
      var auto = (BaseImperativeAuto<?>) selectedField.get(autos);
      robot.robotManager.localization.resetPose(auto.getStartingPoint().getPose());
      DriverStationSim.setEnabled(true);
      var uphill = new HashSet<String>();
      var downhill = new HashSet<String>();
      var visitedStates = new HashSet<String>();
      double waitStartTime = Double.NaN;
      double waitEndTime = Double.NaN;
      Pose2d shootStart = null;
      Pose2d shootEnd = null;
      double deadline = Timer.getFPGATimestamp() + 120.0;
      String previous = "";
      while (!auto.getState().name().equals("DONE") && Timer.getFPGATimestamp() < deadline) {
        robot.tick();
        String state = auto.getState().name();
        visitedStates.add(state);
        var pose = robot.robotManager.localization.getPose();
        var crossing = robot.robotManager.localization.imu.bumpCrossingTracker.getState();
        if (!state.equals(previous)) {
          System.out.println("AUTO " + state + " " + pose + " detector=" + crossing);
          if (state.equals("WAIT_BEHIND_HUB")) {
            waitStartTime = Timer.getFPGATimestamp();
          } else if (previous.equals("WAIT_BEHIND_HUB")) {
            waitEndTime = Timer.getFPGATimestamp();
          }
          previous = state;
        }
        if (crossing == BumpCrossingState.CROSSING_UPHILL) uphill.add(state);
        if (crossing == BumpCrossingState.CROSSING_DOWNHILL) downhill.add(state);
        if (state.equals("SHOOT_1")) {
          if (shootStart == null) {
            shootStart = pose;
          }
          shootEnd = pose;
        }
      }
      assertThat(auto.getState().name()).isEqualTo("DONE");
      assertThat(visitedStates)
          .containsExactlyInAnyOrder(
              "PREPARE_BUMP_OUT_1",
              "CROSS_BUMP_OUT",
              "DRIVE_TO_OPPOSITE_HUB",
              "WAIT_BEHIND_HUB",
              "INTAKE_FIRST_CYCLE",
              "CROSS_BUMP_TO_SHOOT_1",
              "SHOOT_1",
              "DONE");
      assertThat(waitEndTime - waitStartTime).isGreaterThanOrEqualTo(1.0);
      assertThat(uphill).containsExactlyInAnyOrder("CROSS_BUMP_OUT", "CROSS_BUMP_TO_SHOOT_1");
      assertThat(downhill).containsAll(uphill);
      assertThat(shootStart).isNotNull();
      assertThat(shootEnd).isNotNull();
      assertThat(shootEnd.getTranslation().getDistance(shootStart.getTranslation()))
          .as("drives slowly toward tower during SHOOT_1")
          .isGreaterThan(0.5);
    }
  }
}
