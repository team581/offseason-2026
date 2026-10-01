package com.team581.simulation;

import static com.google.common.base.Preconditions.checkArgument;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableList;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import org.java_websocket.WebSocket;
import org.java_websocket.exceptions.WebsocketNotConnectedException;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/** Loopback-only input service. Decode on the socket thread; apply on the robot thread. */
public final class SimulationControlBridge extends WebSocketServer {
  private record Received(SimControlSnapshot frame, long receivedAt, WebSocket owner) {}

  private static final ObjectMapper JSON = new ObjectMapper();
  private static final long TIMEOUT_NANOS = 250_000_000L;

  private static void clearJoysticks() {
    for (int port = 0; port < 6; port++) {
      DriverStationSim.setJoystickButtons(port, 0);
      for (int axis = 0; axis < 12; axis++) {
        DriverStationSim.setJoystickAxis(port, axis, 0);
      }
      for (int pov = 0; pov < 12; pov++) {
        DriverStationSim.setJoystickPOV(port, pov, -1);
      }
      DriverStationSim.setJoystickAxisCount(port, 0);
      DriverStationSim.setJoystickButtonCount(port, 0);
      DriverStationSim.setJoystickPOVCount(port, 0);
    }
  }

  private final AtomicReference<Received> latest = new AtomicReference<>();
  private WebSocket controller;
  private long lastSequence = -1;
  private boolean armed;
  private boolean estopped;
  private final String executable;
  private final String project;

  private boolean wasAttached;

  public SimulationControlBridge(String executable, String project, int port) {
    super(new InetSocketAddress(InetAddress.getLoopbackAddress(), port), 1);
    this.executable = executable;
    this.project = project;
    setConnectionLostTimeout(2);
    setReuseAddr(true);
  }

  /** Called every 20ms by the robot, including while disabled. */
  public void applyLatest() {
    var received = latest.get();
    boolean attached =
        received != null
            && received.owner().isOpen()
            && System.nanoTime() - received.receivedAt() < TIMEOUT_NANOS;
    if (!attached) {
      if (wasAttached || DriverStationSim.getEnabled()) {
        DriverStationSim.setEnabled(false);
        DriverStationSim.setDsAttached(false);
        clearJoysticks();
        DriverStationSim.notifyNewData();
        // HAL's new-data notification marks DS attached; restore disconnected state afterward.
        DriverStationSim.setDsAttached(false);
        DriverStation.refreshData();
      }
      if (received != null && received.owner().isOpen()) {
        received.owner().close(1001, "Control heartbeat timed out");
      }
      wasAttached = false;
      return;
    }
    var frame = received.frame();
    estopped |= frame.estop();
    DriverStationSim.setEnabled(frame.enabled() && !estopped);
    DriverStationSim.setEStop(estopped);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(frame.mode().equals("auto"));
    DriverStationSim.setTest(frame.mode().equals("test"));
    DriverStationSim.setAllianceStationId(AllianceStationID.valueOf(frame.alliance()));
    DriverStationSim.setMatchTime(frame.matchTime());
    for (int port = 0; port < 6; port++) {
      var stick = frame.joysticks().get(port);
      DriverStationSim.setJoystickName(port, stick.name());
      DriverStationSim.setJoystickIsXbox(port, stick.xbox());
      DriverStationSim.setJoystickType(port, stick.xbox() ? 1 : 0);
      DriverStationSim.setJoystickAxisCount(port, stick.axes().size());
      DriverStationSim.setJoystickButtonCount(port, stick.buttons().size());
      DriverStationSim.setJoystickPOVCount(port, stick.povs().size());
      for (int axis = 0; axis < stick.axes().size(); axis++) {
        DriverStationSim.setJoystickAxis(port, axis, stick.axes().get(axis));
      }
      int buttons = 0;
      for (int button = 0; button < stick.buttons().size(); button++) {
        if (stick.buttons().get(button)) {
          buttons |= 1 << button;
        }
      }
      DriverStationSim.setJoystickButtons(port, buttons);
      for (int pov = 0; pov < stick.povs().size(); pov++) {
        DriverStationSim.setJoystickPOV(port, pov, stick.povs().get(pov));
      }
    }
    DriverStationSim.notifyNewData();
    wasAttached = true;
    if (!received.owner().hasBufferedData()) {
      try {
        received
            .owner()
            .send(
                JSON.writeValueAsString(
                    Map.of(
                        "version",
                        1,
                        "sequence",
                        frame.sequence(),
                        "enabled",
                        DriverStationSim.getEnabled(),
                        "estop",
                        estopped,
                        "attached",
                        true,
                        "mode",
                        frame.mode())));
      } catch (IOException | WebsocketNotConnectedException exception) {
        onError(received.owner(), exception);
      }
    }
  }

  @Override
  public synchronized void onClose(WebSocket connection, int code, String reason, boolean remote) {
    if (Objects.equals(connection, controller)) {
      controller = null;
      latest.set(null);
      armed = false;
    }
  }

  @Override
  public void onError(WebSocket connection, Exception exception) {
    System.err.println("Simulation control: " + exception.getMessage());
  }

  @Override
  public synchronized void onMessage(WebSocket connection, String message) {
    if (!Objects.equals(connection, controller)) {
      return;
    }
    try {
      checkArgument(message.length() <= 16_384, "Frame too large");
      var frame = JSON.readValue(message, SimControlSnapshot.class);
      frame.validate();
      checkArgument(frame.sequence() > lastSequence, "Out of order control frame");
      checkArgument(armed || !frame.enabled(), "Send a disabled frame before enabling");
      armed = true;
      lastSequence = frame.sequence();
      latest.set(new Received(frame, System.nanoTime(), connection));
    } catch (IOException | IllegalArgumentException exception) {
      latest.set(null);
      connection.close(1008, "Invalid control frame: " + exception.getMessage());
    }
  }

  @Override
  public synchronized void onOpen(WebSocket connection, ClientHandshake handshake) {
    if (controller != null) {
      connection.close(1008, "Another app controls this simulation");
      return;
    }
    controller = connection;
    armed = false;
    lastSequence = -1;
    latest.set(null);
  }

  @Override
  public void onStart() {
    System.out.println("Simulation control ready at ws://127.0.0.1:" + getPort());
    if (executable == null || executable.isBlank()) {
      return;
    }
    try {
      var command = new ArrayList<String>();
      String path = Path.of(executable).toAbsolutePath().toString();
      if (path.endsWith(".app")) {
        // Launch Services initializes the macOS webview properly. -n lets single-instance
        // forwarding deliver new simulation arguments even when the app is already running.
        command.addAll(ImmutableList.of("/usr/bin/open", "-n", "-a", path, "--args"));
      } else {
        command.add(path);
      }
      command.addAll(
          List.of(
              "--control",
              "ws://127.0.0.1:" + getPort(),
              "--nt",
              "ws://127.0.0.1:5810/nt/simulation-studio",
              "--project",
              project));
      new ProcessBuilder(command).inheritIO().start();
    } catch (IOException exception) {
      System.err.println("Cannot launch Simulation Studio: " + exception.getMessage());
    }
  }
}
