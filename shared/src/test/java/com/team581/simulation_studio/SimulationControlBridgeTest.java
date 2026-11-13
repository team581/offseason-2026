package com.team581.simulation_studio;

import static java.util.Comparator.naturalOrder;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableList;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.URI;
import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class FrameValidationTest {
  @Test
  void rejectsInvalidAxes() {
    var invalid =
        new SimulationControlBridge.Frame(
            1,
            0,
            false,
            false,
            "teleop",
            "Unknown",
            -1,
            Collections.nCopies(
                6,
                new SimulationControlBridge.Frame.Joystick(
                    "x",
                    true,
                    ImmutableList.of(Double.NaN),
                    ImmutableList.of(),
                    ImmutableList.of())));
    assertThatThrownBy(invalid::validate).isInstanceOf(IllegalArgumentException.class);
  }
}

final class SimulationControlBridgeTest {
  private static final class Client extends WebSocketClient {
    private volatile long acknowledged = -1;
    private volatile long receivedAt;

    Client(int port) {
      super(URI.create("ws://127.0.0.1:" + port));
    }

    @Override
    public void onClose(int code, String reason, boolean remote) {}

    @Override
    public void onError(Exception exception) {}

    @Override
    public void onMessage(String message) {
      try {
        acknowledged = JSON.readTree(message).get("sequence").asLong();
        receivedAt = System.nanoTime();
      } catch (java.io.IOException exception) {
        throw new IllegalArgumentException(exception);
      }
    }

    @Override
    public void onOpen(ServerHandshake handshake) {}
  }

  private static final ObjectMapper JSON = new ObjectMapper();

  // Allow cold WebSocket startup on busy CI workers without changing the control watchdog.
  private static final long CONNECT_TIMEOUT_SECONDS = 10;

  public static void main(String[] args) throws Exception {
    HAL.initialize(500, 0);
    var nt = NetworkTableInstance.getDefault();
    nt.startServer("", "127.0.0.1", 1735, 5810);
    var bridge = new SimulationControlBridge(System.getenv("TEAM581_SIM_APP"), "fixture", 5811);
    bridge.start();
    var chooser = new SendableChooser<String>();
    chooser.setDefaultOption("Do nothing", "idle");
    chooser.addOption("Drive", "drive");
    SmartDashboard.putData("Fixture/Auto", chooser);
    nt.getBooleanTopic("/Fixture/FeatureFlags/Example").getEntry(false).set(false);
    nt.getIntegerTopic("/Fixture/Int64").getEntry(0).set(9007199254740993L);
    nt.getStringTopic("/Fixture/Text").getEntry("").set("Hello simulation");
    nt.getDoubleArrayTopic("/Fixture/Array").getEntry(new double[] {}).set(new double[] {1, 2, 3});
    var entries = new ArrayList<DoubleEntry>();
    int count = Integer.parseInt(System.getProperty("fixture.topics", "100"));
    for (int i = 0; i < count; i++) {
      var entry =
          nt.getDoubleTopic((i < 500 ? "/Fixture/Values/Value" : "/Fixture/Metadata/Value") + i)
              .getEntry(0);
      entry.set(i);
      entries.add(entry);
      if (i % 100 == 99) {
        nt.flushLocal();
        Thread.sleep(50);
      }
    }
    Runtime.getRuntime()
        .addShutdownHook(
            new Thread(() -> {
              try {
                bridge.stop(1000);
              } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
              }
              nt.stopServer();
            }));
    long tick = 0;
    while (!Thread.currentThread().isInterrupted()) {
      bridge.applyLatest();
      SmartDashboard.updateValues();
      for (int i = 0; i < Math.min(count, 500); i++) {
        entries.get(i).set(Math.sin(tick * 0.01 + i));
      }
      nt.flush();
      tick++;
      Thread.sleep(20);
    }
  }

  private SimulationControlBridge bridge;
  private Client client;

  private long sequence;

  private long send(boolean enabled, boolean estop) throws Exception {
    long sentSequence = sequence++;
    var stick =
        new SimulationControlBridge.Frame.Joystick(
            "Fixture",
            true,
            ImmutableList.of(0.25, -0.5),
            ImmutableList.of(true, false),
            ImmutableList.of(90));
    client.send(
        JSON.writeValueAsString(
            new SimulationControlBridge.Frame(
                1,
                sentSequence,
                enabled,
                estop,
                "teleop",
                "Red1",
                100,
                Collections.nCopies(6, stick))));
    return sentSequence;
  }

  private void waitUntil(BooleanSupplier condition) throws Exception {
    long deadline = System.nanoTime() + 2_000_000_000L;
    while (!condition.getAsBoolean() && System.nanoTime() < deadline) {
      bridge.applyLatest();
      Thread.sleep(2);
    }
    assertThat(condition.getAsBoolean()).isTrue();
  }

  @Test
  void appliesAllPortsAndDisables() throws Exception {
    send(true, false);
    waitUntil(DriverStationSim::getEnabled);
    for (int port = 0; port < 6; port++) {
      assertThat(DriverStation.getStickAxis(port, 1)).isEqualTo(-0.5);
      assertThat(DriverStation.getStickButton(port, 1)).isTrue();
      assertThat(DriverStation.getStickPOV(port, 0)).isEqualTo(90);
    }
    send(false, false);
    waitUntil(() -> !DriverStationSim.getEnabled());
  }

  @Test
  void disconnectDisablesAndNewControllerMustStartDisabled() throws Exception {
    send(true, false);
    waitUntil(DriverStationSim::getEnabled);
    client.closeBlocking();
    waitUntil(() -> !DriverStationSim.getEnabled());
    client = new Client(bridge.getPort());
    assertThat(client.connectBlocking(CONNECT_TIMEOUT_SECONDS, TimeUnit.SECONDS)).isTrue();
    send(true, false);
    waitUntil(() -> !client.isOpen());
    assertThat(DriverStationSim.getEnabled()).isFalse();
  }

  @Test
  void estopLatchesUntilSimulationRestart() throws Exception {
    send(true, true);
    waitUntil(DriverStationSim::getEStop);
    send(true, false);
    Thread.sleep(20);
    bridge.applyLatest();
    assertThat(DriverStationSim.getEnabled()).isFalse();
    assertThat(DriverStationSim.getEStop()).isTrue();
  }

  @Test
  void measuresBridgeRoundtripOnTwentyMillisecondRobotLoop() throws Exception {
    var executor = java.util.concurrent.Executors.newSingleThreadScheduledExecutor();
    var samples = new java.util.ArrayList<Double>();
    try {
      var unused = executor.scheduleAtFixedRate(bridge::applyLatest, 0, 20, TimeUnit.MILLISECONDS);
      for (int i = 0; i < 100; i++) {
        long sentAt = System.nanoTime();
        long id = send(false, false);
        long deadline = sentAt + 1_000_000_000L;
        while (client.acknowledged < id && System.nanoTime() < deadline) {
          Thread.sleep(1);
        }
        assertThat(client.acknowledged).isGreaterThanOrEqualTo(id);
        samples.add((client.receivedAt - sentAt) / 1_000_000.0);
        Thread.sleep(3);
      }
      samples.sort(naturalOrder());
      System.out.println(
          "Simulation bridge roundtrip p95: "
              + samples.get(94)
              + " ms (100 frames, 20ms apply loop)");
      assertThat(samples.get(94)).isLessThan(40);
    } finally {
      executor.shutdownNow();
      executor.awaitTermination(1, TimeUnit.SECONDS);
    }
  }

  @Test
  void rejectsSecondController() throws Exception {
    var second = new Client(bridge.getPort());
    try {
      second.connectBlocking(CONNECT_TIMEOUT_SECONDS, TimeUnit.SECONDS);
      waitUntil(() -> !second.isOpen());
      assertThat(client.isOpen()).isTrue();
    } finally {
      second.closeBlocking();
    }
  }

  @Test
  void sharedProtocolVectors() throws Exception {
    try (var stream = getClass().getResourceAsStream("/simulation-studio/control-v1.json")) {
      var corpus = JSON.readTree(stream);
      for (var entry : corpus.get("control")) {
        if (entry.get("valid").asBoolean()) {
          JSON.treeToValue(entry.get("frame"), SimulationControlBridge.Frame.class).validate();
        } else {
          assertThatThrownBy(() ->
                  JSON.treeToValue(entry.get("frame"), SimulationControlBridge.Frame.class)
                      .validate())
              .as(entry.get("name").asText())
              .isInstanceOf(Exception.class);
        }
      }
    }
  }

  @BeforeEach
  void start() throws Exception {
    assertThat(HAL.initialize(500, 0)).isTrue();
    DriverStationSim.resetData();
    bridge = new SimulationControlBridge(null, "test", 0);
    bridge.start();
    waitUntil(() -> bridge.getPort() != 0);
    client = new Client(bridge.getPort());
    assertThat(client.connectBlocking(CONNECT_TIMEOUT_SECONDS, TimeUnit.SECONDS)).isTrue();
    send(false, false);
    waitUntil(DriverStationSim::getDsAttached);
  }

  @AfterEach
  void stop() throws Exception {
    if (client != null) {
      client.closeBlocking();
    }
    if (bridge != null) {
      bridge.applyLatest();
      bridge.stop(1000);
    }
    DriverStationSim.resetData();
  }

  @Test
  void watchdogClearsInputsAndDisconnects() throws Exception {
    send(true, false);
    waitUntil(DriverStationSim::getEnabled);
    Thread.sleep(270);
    bridge.applyLatest();
    assertThat(DriverStationSim.getEnabled()).isFalse();
    assertThat(DriverStationSim.getDsAttached()).isFalse();
    assertThat(DriverStation.getStickButton(0, 1)).isFalse();
  }
}
