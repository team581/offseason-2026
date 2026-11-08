package frc.robot.autos.auto_state_machines;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.team581.math.MathHelpers;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
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
import frc.robot.robot_manager.RobotState;
import java.util.ArrayList;
import java.util.HashMap;
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
      String selection = System.getProperty("sim.auto", "LEFT_BUMP_NO_DEPOT");
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
      var stateOrder = new ArrayList<String>();
      boolean right = selection.startsWith("RIGHT");
      boolean sweep = selection.endsWith("_NO_DEPOT");
      double waitStart = Double.NaN;
      double waitEnd = Double.NaN;
      String finalShootingState = "SHOOT_1";
      var stateStartTimes = new HashMap<String, Double>();
      Pose2d shootStart = null;
      Pose2d shootEnd = null;
      double deadline = Timer.getFPGATimestamp() + 120.0;
      String previous = "";
      while (Timer.getFPGATimestamp() < deadline) {
        robot.tick();
        String state = auto.getState().name();
        visitedStates.add(state);
        if (state.equals(finalShootingState)) {
          assertThat(robot.robotManager.getState())
              .as("final score request stays active, even past the old shooting timeout")
              .isIn(RobotState.PREPARE_SCORE, RobotState.SCORE);
          if (stateStartTimes.containsKey(state)
              && Timer.getFPGATimestamp() - stateStartTimes.get(state) >= 7.0) {
            break;
          }
        }
        var pose = robot.robotManager.localization.getPose();
        var crossing = robot.robotManager.localization.imu.bumpCrossingTracker.getState();
        if (!state.equals(previous)) {
          System.out.println("AUTO " + state + " " + pose + " detector=" + crossing);
          if (state.equals("WAIT_BEHIND_HUB")) waitStart = Timer.getFPGATimestamp();
          if (previous.equals("WAIT_BEHIND_HUB")) waitEnd = Timer.getFPGATimestamp();
          stateOrder.add(state);
          stateStartTimes.put(state, Timer.getFPGATimestamp());
          previous = state;
        }
        if (crossing == BumpCrossingState.CROSSING_UPHILL) uphill.add(state);
        if (crossing == BumpCrossingState.CROSSING_DOWNHILL) downhill.add(state);
        if (state.equals("DRIVE_TO_TOWER")) {
          assertThat(robot.robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
        }
        if (state.equals("SHOOT_1")) {
          var goal =
              sweep
                  ? (right
                      ? RightBumpSweepAuto.TOWER_SHOOT_GOAL
                      : LeftBumpSweepAuto.TOWER_SHOOT_GOAL)
                  : (right ? RightBumpAuto.TOWER_SHOOT_GOAL : LeftBumpAuto.TOWER_SHOOT_GOAL);
          assertThat(pose.getTranslation().getDistance(goal.getPose().getTranslation()))
              .as("reaches the tower before shooting")
              .isLessThan(0.12);
          var speeds = robot.robotManager.swerve.getFieldRelativeSpeeds();
          assertThat(MathHelpers.getLinearVelocity(speeds))
              .as("shoots at standstill")
              .isLessThan(0.1);
          assertThat(Math.abs(speeds.omegaRadiansPerSecond)).isLessThan(0.1);
        }
        if (sweep && state.equals("CROSS_BUMP_TO_SHOOT_1")) {
          assertThat(robot.robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
        }
        if (sweep && state.equals("INTAKE_FIRST_CYCLE")) {
          var redPose = FmsUtil.isRedAlliance() ? pose : FieldUtil.pathflip(pose);
          double bumperExtent =
              BumpAutoStart.HALF_BUMPER_METERS
                  * (Math.abs(redPose.getRotation().getCos())
                      + Math.abs(redPose.getRotation().getSin()));
          double laneY = right ? FieldUtil.FIELD_WIDTH_Y - redPose.getY() : redPose.getY();
          assertThat(laneY + bumperExtent)
              .as("sweep keeps the whole robot on its own side of the centerline")
              .isLessThan(FieldUtil.FIELD_WIDTH_Y / 2.0);
        }
        if (state.equals("SHOOT_1")) {
          if (shootStart == null) {
            shootStart = pose;
          }
          shootEnd = pose;
        }
      }
      assertThat(auto.getState().name()).isEqualTo(finalShootingState);
      if (sweep) {
        assertThat(stateOrder)
            .containsExactly(
                "PREPARE_BUMP_OUT_1",
                "CROSS_BUMP_OUT",
                "INTAKE_FIRST_CYCLE",
                "CROSS_BUMP_TO_SHOOT_1",
                "DRIVE_TO_TOWER",
                "SHOOT_1");
        assertThat(robot.robotManager.localization.getPose().getY())
            .isCloseTo(FieldUtil.FIELD_WIDTH_Y / 2.0, offset(0.12));
      } else {
        assertThat(stateOrder)
            .containsExactly(
                "PREPARE_BUMP_OUT_1",
                "CROSS_BUMP_OUT",
                "DRIVE_TO_OPPOSITE_HUB",
                "WAIT_BEHIND_HUB",
                "INTAKE_FIRST_CYCLE",
                "CROSS_BUMP_TO_SHOOT_1",
                "DRIVE_TO_TOWER",
                "SHOOT_1");
        assertThat(waitEnd - waitStart).isGreaterThanOrEqualTo(0.98);
      }
      assertThat(visitedStates)
          .doesNotContain("COLLECT_DEPOT", "DRIVE_TO_DEPOT_FRONT", "SHOOT_BACK_TO_HUB");
      assertThat(uphill).containsExactlyInAnyOrder("CROSS_BUMP_OUT", "CROSS_BUMP_TO_SHOOT_1");
      assertThat(downhill).containsAll(uphill);
      assertThat(shootStart).isNotNull();
      assertThat(shootEnd).isNotNull();
      assertThat(shootEnd.getTranslation().getDistance(shootStart.getTranslation()))
          .as("holds still during SHOOT_1")
          .isLessThan(0.1);
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      robot.tick();
      assertThat(auto.shouldRun()).as("auto stops running when disabled").isFalse();
      DriverStationSim.setAutonomous(false);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.tick();
      assertThat(auto.shouldRun()).as("auto stops running in teleop").isFalse();
    }
  }
}
