package com.team581.simulation;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

/** Real HAL and NT4 server for desktop smoke tests and repeatable load measurements. */
public final class SimulationAppFixture {
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
            new Thread(
                () -> {
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

  private SimulationAppFixture() {}
}
