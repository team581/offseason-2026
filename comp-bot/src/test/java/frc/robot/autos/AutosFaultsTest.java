package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.CompletableFuture;
import org.junit.jupiter.api.Test;

final class AutosFaultsTest {
  @Test
  void logsAndClearsWarmupAndPathCalculationFaults() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var robotManager =
        TrailblazerTrenchAutoTest.testContext(TestTrailblazer.create(), Alliance.Blue).robot();
    var nt = NetworkTableInstance.getDefault();
    StringArraySubscriber warnings =
        nt.getStringArrayTopic("/SmartDashboard/Alerts/warnings").subscribe(new String[0]);
    StringPublisher autoSelected =
        nt.getStringTopic("/SmartDashboard/Autos/SelectedAuto/selected").publish();

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    var warmupFuture = new CompletableFuture<Void>();
    Queue<Runnable> queuedPathTasks = new ArrayDeque<>();
    var autos =
        new Autos(robotManager, TestTrailblazer.create(), queuedPathTasks::add, warmupFuture);

    SmartDashboard.updateValues();
    autoSelected.set(AutoSelection.TRAILBLAZER_3_RIGHT_TRENCH.toString());
    nt.flush();
    SmartDashboard.updateValues();
    autos.periodic();
    SmartDashboard.updateValues();

    assertThat(warnings.get())
        .containsExactlyInAnyOrder(
            "Trailblazer warmup still running", "Trailblazer path calculation still running");

    warmupFuture.complete(null);
    autos.periodic();
    SmartDashboard.updateValues();

    assertThat(warnings.get())
        .containsExactly("Trailblazer path calculation still running")
        .doesNotContain("Trailblazer warmup still running");

    while (!queuedPathTasks.isEmpty()) {
      queuedPathTasks.poll().run();
    }
    autos.periodic();
    SmartDashboard.updateValues();

    assertThat(warnings.get())
        .doesNotContain(
            "Trailblazer warmup still running", "Trailblazer path calculation still running");
  }

  @Test
  void realWarmupCompletesAndClearsWarmupFault() throws Exception {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var robotManager =
        TrailblazerTrenchAutoTest.testContext(TestTrailblazer.create(), Alliance.Blue).robot();
    var nt = NetworkTableInstance.getDefault();
    StringArraySubscriber warnings =
        nt.getStringArrayTopic("/SmartDashboard/Alerts/warnings").subscribe(new String[0]);

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    var autos = new Autos(robotManager, TestTrailblazer.create());
    long deadlineNanos = System.nanoTime() + 10_000_000_000L;
    while (System.nanoTime() < deadlineNanos) {
      autos.periodic();
      SmartDashboard.updateValues();
      boolean warmupRunning = false;
      for (String warning : warnings.get()) {
        if (warning.equals("Trailblazer warmup still running")) {
          warmupRunning = true;
          break;
        }
      }
      if (!warmupRunning) {
        break;
      }
      Thread.sleep(10);
    }

    assertThat(warnings.get()).doesNotContain("Trailblazer warmup still running");
  }

  @Test
  void seedsTrailblazerTelemetryKeysBeforeEnable() throws Exception {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var robotManager =
        TrailblazerTrenchAutoTest.testContext(TestTrailblazer.create(), Alliance.Blue).robot();
    var nt = NetworkTableInstance.getDefault();
    var statusSub = nt.getStringTopic("/Robot/Trailblazer3/Status").subscribe("UNSET");
    var speedSub = nt.getDoubleTopic("/Robot/Trailblazer3/CommandedSpeed").subscribe(-1.0);
    var diagSub = nt.getStringTopic("/Robot/Trailblazer3/Diagnostic").subscribe("UNSET");

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    new Autos(
        robotManager,
        TestTrailblazer.create(),
        Runnable::run,
        CompletableFuture.completedFuture(null));

    long deadlineNanos = System.nanoTime() + 2_000_000_000L;
    while (System.nanoTime() < deadlineNanos) {
      nt.flush();
      if ("IDLE".equals(statusSub.get()) && speedSub.get() == 0.0 && "".equals(diagSub.get())) {
        break;
      }
      Thread.sleep(10);
    }

    assertThat(statusSub.get()).isEqualTo("IDLE");
    assertThat(speedSub.get()).isEqualTo(0.0);
    assertThat(diagSub.get()).isEmpty();
  }
}
