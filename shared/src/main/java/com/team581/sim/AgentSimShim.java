package com.team581.sim;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

/**
 * Simulation-only bridge that lets an external agent drive the robot over NetworkTables.
 *
 * <p>When {@code /Agent/Enabled} is true, the shim re-asserts the driver joystick and DriverStation
 * state from NT topics every robot loop via {@link DriverStationSim}, so the agent wins over the
 * Sim GUI DriverStation. When false, the shim is inert (and forces the robot disabled once, so a
 * disconnecting agent never leaves the robot enabled).
 *
 * <p>Never construct this on a real robot.
 */
public final class AgentSimShim {
  /** Xbox axis count: LeftX, LeftY, LeftTrigger, RightTrigger, RightX, RightY. */
  private static final int AXIS_COUNT = 6;

  /** Xbox button count: A, B, X, Y, LB, RB, Back, Start, LeftStick, RightStick. */
  private static final int BUTTON_COUNT = 10;

  private static final int DRIVER_JOYSTICK_PORT = 0;

  private final BooleanSubscriber enabledSub;
  private final DoubleArraySubscriber axesSub;
  private final BooleanArraySubscriber buttonsSub;
  private final BooleanSubscriber dsEnabledSub;
  private final StringSubscriber modeSub;
  private final StringSubscriber allianceSub;
  private final BooleanPublisher activePub;
  private final BooleanPublisher robotEnabledPub;

  private boolean wasActive = false;

  public AgentSimShim() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Agent");

    enabledSub = table.getBooleanTopic("Enabled").subscribe(false);
    axesSub = table.getDoubleArrayTopic("Joystick/Axes").subscribe(new double[AXIS_COUNT]);
    buttonsSub =
        table.getBooleanArrayTopic("Joystick/Buttons").subscribe(new boolean[BUTTON_COUNT]);
    dsEnabledSub = table.getBooleanTopic("DriverStation/Enabled").subscribe(false);
    modeSub = table.getStringTopic("DriverStation/Mode").subscribe("teleop");
    allianceSub = table.getStringTopic("DriverStation/AllianceStation").subscribe("");
    activePub = table.getBooleanTopic("Status/Active").publish();
    robotEnabledPub = table.getBooleanTopic("Status/RobotEnabled").publish();
  }

  public void periodic() {
    var active = enabledSub.get();
    activePub.set(active);
    // Echo the robot's real DS state so the agent can tell when its enable writes
    // have actually taken effect (robot-side NT setup lags a fresh connection).
    robotEnabledPub.set(DriverStation.isEnabled());

    if (!active) {
      if (wasActive) {
        // Agent handed control back: leave the robot in a safe state.
        DriverStationSim.setEnabled(false);
        for (int axis = 0; axis < AXIS_COUNT; axis++) {
          DriverStationSim.setJoystickAxis(DRIVER_JOYSTICK_PORT, axis, 0.0);
        }
        for (int button = 1; button <= BUTTON_COUNT; button++) {
          DriverStationSim.setJoystickButton(DRIVER_JOYSTICK_PORT, button, false);
        }
        DriverStationSim.notifyNewData();
      }
      wasActive = false;
      return;
    }
    wasActive = true;

    var axes = axesSub.get();
    var buttons = buttonsSub.get();

    DriverStationSim.setJoystickAxisCount(DRIVER_JOYSTICK_PORT, AXIS_COUNT);
    DriverStationSim.setJoystickButtonCount(DRIVER_JOYSTICK_PORT, BUTTON_COUNT);
    DriverStationSim.setJoystickIsXbox(DRIVER_JOYSTICK_PORT, true);
    DriverStationSim.setJoystickName(DRIVER_JOYSTICK_PORT, "Agent Gamepad");

    for (int axis = 0; axis < AXIS_COUNT; axis++) {
      var value = axis < axes.length ? MathUtil.clamp(axes[axis], -1.0, 1.0) : 0.0;
      DriverStationSim.setJoystickAxis(DRIVER_JOYSTICK_PORT, axis, value);
    }
    for (int button = 1; button <= BUTTON_COUNT; button++) {
      var pressed = button - 1 < buttons.length && buttons[button - 1];
      DriverStationSim.setJoystickButton(DRIVER_JOYSTICK_PORT, button, pressed);
    }

    var alliance = allianceSub.get();
    if (!alliance.isEmpty()) {
      try {
        DriverStationSim.setAllianceStationId(AllianceStationID.valueOf(alliance));
      } catch (IllegalArgumentException e) {
        // Ignore malformed alliance station strings.
      }
    }

    var mode = modeSub.get();
    DriverStationSim.setTest(mode.equals("test"));
    DriverStationSim.setAutonomous(mode.equals("autonomous"));
    DriverStationSim.setEnabled(dsEnabledSub.get());

    DriverStationSim.notifyNewData();
  }
}
