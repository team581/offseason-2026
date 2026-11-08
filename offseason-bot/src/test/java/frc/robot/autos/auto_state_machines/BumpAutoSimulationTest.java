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
      var stateOrder = new ArrayList<String>();
      boolean depotAuto = selection.equals("LEFT_BUMP_DEPOT");
      String finalShootingState = depotAuto ? "SHOOT_BACK_TO_HUB" : "SHOOT_1";
      var stateStartTimes = new HashMap<String, Double>();
      var stateEndTimes = new HashMap<String, Double>();
      Pose2d firstScorePose = null;
      Pose2d finalShootStart = null;
      Pose2d finalShootEnd = null;
      double waitStartTime = Double.NaN;
      double waitEndTime = Double.NaN;
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
          if (state.equals("WAIT_BEHIND_HUB")) {
            waitStartTime = Timer.getFPGATimestamp();
          } else if (previous.equals("WAIT_BEHIND_HUB")) {
            waitEndTime = Timer.getFPGATimestamp();
          }
          stateOrder.add(state);
          stateStartTimes.put(state, Timer.getFPGATimestamp());
          stateEndTimes.put(previous, Timer.getFPGATimestamp());
          previous = state;
        }
        if (crossing == BumpCrossingState.CROSSING_UPHILL) uphill.add(state);
        if (crossing == BumpCrossingState.CROSSING_DOWNHILL) downhill.add(state);
        if (state.equals("DRIVE_TO_TOWER")) {
          assertThat(robot.robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
        }
        if (state.equals("SHOOT_1")) {
          var goal =
              depotAuto
                  ? LeftBumpDepotAuto.DEPOT_APPROACH
                  : selection.equals("RIGHT_BUMP_CROSS")
                      ? RightBumpAuto.TOWER_SHOOT_GOAL
                      : LeftBumpAuto.TOWER_SHOOT_GOAL;
          assertThat(pose.getTranslation().getDistance(goal.getPose().getTranslation()))
              .as("reaches the tower before shooting")
              .isLessThan(0.12);
          var speeds = robot.robotManager.swerve.getFieldRelativeSpeeds();
          assertThat(MathHelpers.getLinearVelocity(speeds))
              .as("shoots at standstill")
              .isLessThan(0.1);
          assertThat(Math.abs(speeds.omegaRadiansPerSecond)).isLessThan(0.1);
        }
        if (depotAuto
            && (state.equals("CROSS_BUMP_TO_SHOOT_1") || state.equals("DRIVE_TO_DEPOT_FRONT"))) {
          assertThat(robot.robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
        }
        if (depotAuto && state.equals("INTAKE_FIRST_CYCLE")) {
          var redPose = FmsUtil.isRedAlliance() ? pose : FieldUtil.pathflip(pose);
          double bumperExtent =
              BumpAutoStart.HALF_BUMPER_METERS
                  * (Math.abs(redPose.getRotation().getCos())
                      + Math.abs(redPose.getRotation().getSin()));
          assertThat(redPose.getY() + bumperExtent)
              .as("sweep keeps the whole robot left of the longitudinal centerline")
              .isLessThan(FieldUtil.FIELD_WIDTH_Y / 2.0);
          if (stateOrder.get(stateOrder.size() - 1).equals("INTAKE_FIRST_CYCLE")
              && Timer.getFPGATimestamp() - stateStartTimes.get(state) < 0.5) {
            assertThat(redPose.getY()).isLessThan(FieldUtil.RED_DEPOT_BUMP_CENTER.getY() + 0.1);
          }
        }
        if (depotAuto
            && state.equals("SHOOT_1")
            && firstScorePose == null
            && robot.robotManager.getState() == RobotState.SCORE) {
          firstScorePose = pose;
          System.out.println("First actual SCORE pose " + pose);
        }
        if (state.equals("SHOOT_BACK_TO_HUB")) {
          if (finalShootStart == null) finalShootStart = pose;
          finalShootEnd = pose;
        }
        if (depotAuto && state.equals("COLLECT_DEPOT")) {
          var redPose = FmsUtil.isRedAlliance() ? pose : FieldUtil.pathflip(pose);
          assertThat(redPose.getRotation().getDegrees()).isCloseTo(-75.0, offset(4.0));
        }
        if (state.equals("SHOOT_1")) {
          if (shootStart == null) {
            shootStart = pose;
          }
          shootEnd = pose;
        }
      }
      assertThat(auto.getState().name()).isEqualTo(finalShootingState);
      if (depotAuto) {
        for (var entry : stateStartTimes.entrySet()) {
          if (stateEndTimes.containsKey(entry.getKey())) {
            System.out.println(
                "Depot phase "
                    + entry.getKey()
                    + " seconds="
                    + (stateEndTimes.get(entry.getKey()) - entry.getValue()));
          }
        }
        assertThat(stateOrder)
            .containsExactly(
                "PREPARE_BUMP_OUT_1",
                "CROSS_BUMP_OUT",
                "INTAKE_FIRST_CYCLE",
                "CROSS_BUMP_TO_SHOOT_1",
                "DRIVE_TO_TOWER",
                "SHOOT_1",
                "COLLECT_DEPOT",
                "DRIVE_TO_DEPOT_FRONT",
                "SHOOT_BACK_TO_HUB");
        assertThat(stateEndTimes.get("SHOOT_1") - stateStartTimes.get("SHOOT_1"))
            .isBetween(4.95, 6.5);
        assertThat(Timer.getFPGATimestamp() - stateStartTimes.get("SHOOT_BACK_TO_HUB"))
            .isGreaterThanOrEqualTo(7.0);
        assertThat(firstScorePose).as("scores after stopping by the tower").isNotNull();
        assertThat(
                firstScorePose
                    .getTranslation()
                    .getDistance(LeftBumpDepotAuto.DEPOT_APPROACH.getPose().getTranslation()))
            .isLessThan(0.12);
        assertThat(
                stateEndTimes.get("DRIVE_TO_DEPOT_FRONT")
                    - stateStartTimes.get("DRIVE_TO_DEPOT_FRONT"))
            .as("clears the depot quickly before starting the second shot")
            .isLessThan(2.0);
        assertThat(finalShootStart).isNotNull();
        assertThat(
                finalShootStart
                    .getTranslation()
                    .getDistance(LeftBumpDepotAuto.DEPOT_FRONT.getPose().getTranslation()))
            .as("second shot only starts after reaching the front of the depot")
            .isLessThan(0.12);
        assertThat(finalShootEnd).isNotNull();
        assertThat(finalShootEnd.getTranslation().getDistance(finalShootStart.getTranslation()))
            .isGreaterThan(0.35);
        var expected = LeftBumpDepotAuto.FINAL_SHOOT_GOAL.getPose();
        assertThat(
                robot
                    .robotManager
                    .localization
                    .getPose()
                    .getTranslation()
                    .getDistance(expected.getTranslation()))
            .isLessThan(0.15);
        var stopPose = robot.robotManager.localization.getPose();
        double holdUntil = Timer.getFPGATimestamp() + 0.5;
        robot.runUntil(() -> Timer.getFPGATimestamp() >= holdUntil, 1.0);
        assertThat(
                robot
                    .robotManager
                    .localization
                    .getPose()
                    .getTranslation()
                    .getDistance(stopPose.getTranslation()))
            .isLessThan(0.1);
      } else {
        assertThat(visitedStates)
            .containsExactlyInAnyOrder(
                "PREPARE_BUMP_OUT_1",
                "CROSS_BUMP_OUT",
                "DRIVE_TO_OPPOSITE_HUB",
                "WAIT_BEHIND_HUB",
                "INTAKE_FIRST_CYCLE",
                "CROSS_BUMP_TO_SHOOT_1",
                "DRIVE_TO_TOWER",
                "SHOOT_1");
        // States are sampled after each tick, so allow one 20 ms loop of timing skew.
        assertThat(waitEndTime - waitStartTime).isGreaterThanOrEqualTo(0.98);
      }
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
