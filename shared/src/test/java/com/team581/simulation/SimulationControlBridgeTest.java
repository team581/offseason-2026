package com.team581.simulation;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.net.URI;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

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
  private SimulationControlBridge bridge;
  private Client client;

  private long sequence;

  private long send(boolean enabled, boolean estop) throws Exception {
    long sentSequence = sequence++;
    var stick =
        new SimControlSnapshot.Joystick(
            "Fixture", true, List.of(0.25, -0.5), List.of(true, false), List.of(90));
    client.send(
        JSON.writeValueAsString(
            new SimControlSnapshot(
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
    assertThat(client.connectBlocking(2, TimeUnit.SECONDS)).isTrue();
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
      executor.scheduleAtFixedRate(bridge::applyLatest, 0, 20, TimeUnit.MILLISECONDS);
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
      samples.sort(Double::compareTo);
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
  void rejectsInvalidAxesWithoutRefreshingWatchdog() {
    var invalid =
        new SimControlSnapshot(
            1,
            0,
            false,
            false,
            "teleop",
            "Unknown",
            -1,
            Collections.nCopies(
                6,
                new SimControlSnapshot.Joystick(
                    "x", true, List.of(Double.NaN), List.of(), List.of())));
    assertThatThrownBy(invalid::validate).isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void rejectsSecondController() throws Exception {
    var second = new Client(bridge.getPort());
    try {
      second.connectBlocking(2, TimeUnit.SECONDS);
      waitUntil(() -> !second.isOpen());
      assertThat(client.isOpen()).isTrue();
    } finally {
      second.closeBlocking();
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
    assertThat(client.connectBlocking(2, TimeUnit.SECONDS)).isTrue();
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
