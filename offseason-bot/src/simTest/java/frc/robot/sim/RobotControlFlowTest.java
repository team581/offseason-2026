package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.Base581Robot;
import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.Robot;
import frc.robot.autos.Autos;
import frc.robot.config.DSOptions;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.FeederState;
import frc.robot.funneler.FunnelerState;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.hopper_manager.HopperCapacity;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.robot_manager.hopper_manager.HopperState;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Exercises production manager transitions and controller bindings with simulated hardware. */
final class RobotControlFlowTest {
  private static Robot robot;
  private static RobotManager manager;
  private static HopperManager hopper;
  private static EventLoop bindings;

  private static Field field(Class<?> type, String name) throws NoSuchFieldException {
    var field = type.getDeclaredField(name);
    field.setAccessible(true);
    return field;
  }

  private static void publishDriverStation() {
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void transition(StateMachine<?> machine, Enum<?> state)
      throws ReflectiveOperationException {
    var method = StateMachine.class.getDeclaredMethod("setStateFromRequest", Enum.class);
    method.setAccessible(true);
    method.invoke(machine, state);
  }

  @AfterAll
  static void closeRobot() {
    DriverStationSim.setEnabled(false);
    publishDriverStation();
    robot.close();
  }

  @BeforeAll
  static void initialize() throws ReflectiveOperationException {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    robot = new Robot();
    // Initialize dashboard defaults before tests deliberately override camera availability.
    assertThat(DSOptions.USE_TAG_LIMELIGHTS.get()).isTrue();
    manager = (RobotManager) field(Robot.class, "robotManager").get(robot);
    hopper = manager.hopperManager;
    bindings = (EventLoop) field(Base581Robot.class, "buttonBindingsLoop").get(robot);
  }

  @Test
  void fallbackFeedStopsTransportWhenReadinessIsLost() throws ReflectiveOperationException {
    NetworkTableInstance.getDefault()
        .getTable("DSOptions")
        .getEntry("UseTagLimelights")
        .setBoolean(false);
    manager.prepareFeedRequest();
    assertThat(manager.getState()).isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
    transition(manager, RobotState.FALLBACK_FEED);
    hopper.feedRequest();
    assertThat(hopper.feeder.getState()).isEqualTo(FeederState.FEED);

    manager.robotPeriodic();

    assertThat(manager.getState()).isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(hopper.feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void fallbackScoreStopsTransportWhenReadinessIsLost() throws ReflectiveOperationException {
    NetworkTableInstance.getDefault()
        .getTable("DSOptions")
        .getEntry("UseTagLimelights")
        .setBoolean(false);
    manager.prepareScoreRequest();
    assertThat(manager.getState()).isEqualTo(RobotState.PREPARE_FALLBACK_SCORE);
    transition(manager, RobotState.FALLBACK_SCORE);
    hopper.scoreRequest();
    assertThat(hopper.feeder.getState()).isEqualTo(FeederState.SCORE);
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.SHOOT);

    // The unhomed mechanisms are not ready, so the real state machine must return to preparation.
    manager.robotPeriodic();

    assertThat(manager.getState()).isEqualTo(RobotState.PREPARE_FALLBACK_SCORE);
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
    assertThat(hopper.feeder.getState()).isEqualTo(FeederState.IDLE);
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void funnelerChangesFromBallFillingToIdleWhenTowerFills() throws ReflectiveOperationException {
    // Seed the sensor inputs needed for tower filling without running motor physics.
    field(Deploy.class, "leftMotorPosition")
        .setDouble(hopper.deploy, DeployState.INTAKE.getLength());
    field(Deploy.class, "rightMotorPosition")
        .setDouble(hopper.deploy, DeployState.INTAKE.getLength());
    field(HopperManager.class, "hopperCapacity").set(hopper, HopperCapacity.MEDIUM);
    ((Timer) field(HopperManager.class, "canRangeUpdateTimer").get(hopper)).restart();
    hopper.robotPeriodic();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.BALL_FILLING);
    field(HopperManager.class, "towerSensorDebounced").setBoolean(hopper, true);
    hopper.robotPeriodic();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.IDLE);
    field(HopperManager.class, "towerSensorDebounced").setBoolean(hopper, false);
    field(Deploy.class, "leftMotorPosition").setDouble(hopper.deploy, 0.0);
    field(Deploy.class, "rightMotorPosition").setDouble(hopper.deploy, 0.0);
  }

  @Test
  void funnelerFollowsTransportDuringIntakeShootingAndEject() {
    hopper.setDriverWantsIntake(true);
    hopper.idleRequest();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.INTAKE);
    hopper.scoreRequest();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.SHOOT);
    hopper.feedRequest();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.SHOOT);
    hopper.setDriverWantsIntake(false);
    hopper.setDriverWantsEject(true);
    hopper.idleRequest();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.IDLE);
    hopper.setDriverWantsEject(false);
    hopper.idleRequest();
    assertThat(hopper.funneler.getState()).isEqualTo(FunnelerState.IDLE);
  }

  @Test
  void releasingEitherStowControlRestoresDeployedIdle() {
    var driver = new JoystickSim(0);
    var operator = new JoystickSim(1);
    driver.setButtonCount(10);
    driver.setAxisCount(6);
    operator.setButtonCount(10);
    operator.setAxisCount(6);
    DriverStationSim.setEnabled(true);
    publishDriverStation();
    bindings.poll();

    driver.setRawButton(6, true);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_STOWED);
    driver.setRawButton(6, false);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);

    operator.setRawAxis(2, 1.0);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_STOWED);
    operator.setRawAxis(2, 0.0);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
  }

  @BeforeEach
  void resetRequests() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    publishDriverStation();
    bindings.poll();
    NetworkTableInstance.getDefault()
        .getTable("DSOptions")
        .getEntry("UseTagLimelights")
        .setBoolean(true);
    manager.idleRequest();
    manager.cancelIntakeRequest();
    manager.cancelStowDeployRequest();
    hopper.setDriverWantsEject(false);
    hopper.idleRequest();
  }

  @Test
  void robotRegistersAutosAndHomingUsesTheCorrectDirection() throws ReflectiveOperationException {
    assertThat(field(Robot.class, "autos").get(robot)).isInstanceOf(Autos.class);
    Deploy deploy = hopper.deploy;
    DriverStationSim.setAutonomous(true);
    publishDriverStation();
    deploy.homingRequest();
    assertThat(deploy.getState()).isEqualTo(DeployState.HOME_INWARD);
    DriverStationSim.setAutonomous(false);
    publishDriverStation();
    deploy.homingRequest();
    assertThat(deploy.getState()).isEqualTo(DeployState.HOME_OUTWARD);
    transition(deploy, DeployState.UNHOMED);
  }

  @Test
  void stowRemainsRequestedUntilBothControlsAreReleased() {
    var driver = new JoystickSim(0);
    var operator = new JoystickSim(1);
    driver.setButtonCount(10);
    driver.setAxisCount(6);
    operator.setButtonCount(10);
    operator.setAxisCount(6);
    DriverStationSim.setEnabled(true);
    driver.setRawButton(6, true);
    operator.setRawAxis(2, 1.0);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_STOWED);
    driver.setRawButton(6, false);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_STOWED);
    operator.setRawAxis(2, 0.0);
    publishDriverStation();
    bindings.poll();
    hopper.idleRequest();
    assertThat(hopper.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
  }
}
