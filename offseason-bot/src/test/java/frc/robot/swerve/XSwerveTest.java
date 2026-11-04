package frc.robot.swerve;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.HeadlessRunner;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

final class XSwerveTest {
  private static HeadlessRunner robot;

  private static boolean isModulesInX(SwerveModuleState[] states) {
    if (states == null || states.length != 4) {
      return false;
    }
    var locations = robot.robotManager.swerve.drivetrain.getModuleLocations();
    for (int i = 0; i < states.length; i++) {
      var expectedAngle = locations[i].getAngle();
      boolean aligned =
          MathUtil.isNear(expectedAngle.getDegrees(), states[i].angle.getDegrees(), 5.0, -180, 180)
              || MathUtil.isNear(
                  expectedAngle.getDegrees() + 180.0, states[i].angle.getDegrees(), 5.0, -180, 180);
      if (!aligned || !MathUtil.isNear(0, states[i].speedMetersPerSecond, 0.05)) {
        return false;
      }
    }
    return true;
  }

  @AfterAll
  static void closeRobot() {
    if (robot != null) {
      robot.close();
    }
  }

  @BeforeAll
  static void initializeRobot() {
    assertThat(HAL.initialize(500, 0)).as("Initialize desktop HAL").isTrue();
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.0);
    robot = new HeadlessRunner();
  }

  @Test
  @Timeout(60)
  void swerveXesWhenStoppedInScoreAndFeedStates() {
    var driverSim = new XboxControllerSim(robot.robotManager.driverController);
    try (var resetPose =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("ResetPoseForAuto")
                .publish();
        var testMode =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("TestMode")
                .publish()) {
      resetPose.set(false);
      testMode.set(false);
      driverSim.setLeftX(0.0);
      driverSim.setLeftY(0.0);
      driverSim.setRightX(0.0);

      DriverStationSim.setEnabled(false);
      robot.tick();
      robot.robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
      DriverStationSim.setEnabled(true);
      robot.robotManager.idleRequest();

      // Drive forward briefly so all modules align to 0 degrees instead of an X.
      driverSim.setLeftY(-0.5);
      double driveUntil = Timer.getFPGATimestamp() + 0.3;
      robot.runUntil(() -> Timer.getFPGATimestamp() >= driveUntil, 2.0);
      driverSim.setLeftY(0.0);

      // In IDLE (MANUAL swerve state), stopping must not X the modules even after the debounce.
      double idleWaitUntil = Timer.getFPGATimestamp() + 0.5;
      robot.runUntil(() -> Timer.getFPGATimestamp() >= idleWaitUntil, 2.0);
      assertThat(isModulesInX(robot.robotManager.swerve.getDriveState().ModuleTargets))
          .as("Swerve does not X while in MANUAL state")
          .isFalse();

      for (Runnable enterState :
          new Runnable[] {
            robot.robotManager::prepareScoreRequest, robot.robotManager::prepareFeedRequest
          }) {
        // While driver is requesting speeds in score/feed, swerve must drive and not X.
        driverSim.setLeftY(-0.5);
        enterState.run();
        double movingUntil = Timer.getFPGATimestamp() + 0.5;
        robot.runUntil(() -> Timer.getFPGATimestamp() >= movingUntil, 2.0);
        assertThat(isModulesInX(robot.robotManager.swerve.getDriveState().ModuleTargets))
            .as("Swerve does not X while driver is requesting speeds")
            .isFalse();

        // When driver stops inputting speeds, swerve Xes after the debounce and modules settle.
        driverSim.setLeftY(0.0);
        robot.runUntil(
            () ->
                isModulesInX(robot.robotManager.swerve.getDriveState().ModuleTargets)
                    && isModulesInX(robot.robotManager.swerve.getDriveState().ModuleStates),
            3.0);

        // Inputting speeds again exits X-swerve.
        driverSim.setLeftY(-0.5);
        robot.runUntil(
            () -> !isModulesInX(robot.robotManager.swerve.getDriveState().ModuleTargets), 0.2);
        driverSim.setLeftY(0.0);
      }
    } finally {
      driverSim.setLeftX(0.0);
      driverSim.setLeftY(0.0);
      driverSim.setRightX(0.0);
      robot.robotManager.idleRequest();
      DriverStationSim.setEnabled(false);
      robot.tick();
    }
  }
}
