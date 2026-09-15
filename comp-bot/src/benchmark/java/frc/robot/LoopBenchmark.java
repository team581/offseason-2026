package frc.robot;

import static java.nio.charset.StandardCharsets.UTF_8;

import com.team581.signals.Signals;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.LockSupport;

/** Real-time-paced desktop benchmark for the complete comp-bot periodic path. */
public final class LoopBenchmark {
  private static final double LOOP_PERIOD_SECONDS = 0.020;
  private static final long LOOP_PERIOD_NANOS = 20_000_000L;
  private static final int WARMUP_LOOPS = 1_000;

  public static void main(String[] args) throws IOException, NoSuchAlgorithmException {
    if (args.length != 3) {
      throw new IllegalArgumentException(
          "Usage: LoopBenchmark <auto|teleop> <seconds> <output.json>");
    }
    String mode = args[0];
    int measurementSeconds = Integer.parseInt(args[1]);
    Path output = Path.of(args[2]);

    if (!HAL.initialize(500, 0)) {
      throw new IllegalStateException("HAL failed to initialize");
    }
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    Robot robot = new Robot();
    robot.robotInit();
    // Phoenix cannot reproduce the timing of a real CAN/CAN-FD refresh on a laptop. Seed cached
    // values below, and benchmark the CPU-controlled part of the loop without native timeout noise.
    Signals.setRefreshEnabledForBenchmark(false);
    robot.benchmarkInitializeSimulation();
    DIOSim towerSensor = new DIOSim(9);
    IntegerSubscriber queueRemaining =
        NetworkTableInstance.getDefault()
            .getIntegerTopic("/Robot/DogLog/QueueRemainingCapacity")
            .subscribe(4096);
    StringArraySubscriber seenFaults =
        NetworkTableInstance.getDefault()
            .getStringArrayTopic("/Robot/Faults/Seen")
            .subscribe(new String[0]);
    AtomicLong minimumQueueRemaining = new AtomicLong(4096);
    runLoops(robot, towerSensor, queueRemaining, minimumQueueRemaining, WARMUP_LOOPS, null, null);

    boolean autonomous =
        switch (mode) {
          case "auto" -> true;
          case "teleop" -> false;
          default -> throw new IllegalArgumentException("Unknown benchmark mode: " + mode);
        };
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickAxis(0, 0, 0.35);
    DriverStationSim.setJoystickAxis(0, 1, -0.55);
    DriverStationSim.setJoystickAxis(0, 4, 0.20);
    DriverStationSim.notifyNewData();
    if (autonomous) {
      robot.autonomousInit();
    } else {
      robot.teleopInit();
    }

    List<Long> durations = new ArrayList<>(measurementSeconds * 50);
    MessageDigest behaviorDigest = MessageDigest.getInstance("SHA-256");
    long heapBefore = usedHeapBytes();
    runLoops(
        robot,
        towerSensor,
        queueRemaining,
        minimumQueueRemaining,
        measurementSeconds * 50,
        durations,
        behaviorDigest);
    long heapAfter = usedHeapBytes();

    durations.sort(Long::compare);
    String json =
        reportJson(
            mode,
            durations,
            behaviorDigest.digest(),
            minimumQueueRemaining.get(),
            hasDogLogQueueFault(seenFaults.get()),
            heapBefore,
            heapAfter);
    Files.createDirectories(output.getParent());
    Files.writeString(output, json, UTF_8);

    // Vendor libraries own process-wide executors that are not part of the measured loop. A fresh
    // JVM is required for every run, so terminate after artifacts are safely written.
    System.exit(0);
  }

  private static boolean hasDogLogQueueFault(String[] seenFaults) {
    for (String fault : seenFaults) {
      if (fault.contains("MAX_QUEUED_LOGS") || fault.contains("QUEUE_RESIZE_DROPPED_LOGS")) {
        return true;
      }
    }
    return false;
  }

  private static double percentile(List<Long> sorted, double fraction) {
    if (sorted.isEmpty()) {
      return Double.NaN;
    }
    double position = (sorted.size() - 1) * fraction;
    int lower = (int) Math.floor(position);
    int upper = (int) Math.ceil(position);
    double nanos =
        sorted.get(lower) * (1.0 - (position - lower)) + sorted.get(upper) * (position - lower);
    return nanos / 1e9;
  }

  private static String reportJson(
      String mode,
      List<Long> sorted,
      byte[] behaviorDigest,
      long minimumQueueRemaining,
      boolean dogLogQueueFaultSeen,
      long heapBefore,
      long heapAfter) {
    double mean = sorted.stream().mapToLong(Long::longValue).average().orElse(Double.NaN) / 1e9;
    return "{\n"
        + "  \"schema_version\": 1,\n"
        + "  \"mode\": \""
        + mode
        + "\",\n"
        + "  \"samples\": "
        + sorted.size()
        + ",\n"
        + "  \"mean\": "
        + mean
        + ",\n"
        + "  \"p50\": "
        + percentile(sorted, 0.50)
        + ",\n"
        + "  \"p95\": "
        + percentile(sorted, 0.95)
        + ",\n"
        + "  \"p99\": "
        + percentile(sorted, 0.99)
        + ",\n"
        + "  \"max\": "
        + sorted.get(sorted.size() - 1) / 1e9
        + ",\n"
        + "  \"overruns_20ms\": "
        + sorted.stream().filter(value -> value > LOOP_PERIOD_NANOS).count()
        + ",\n"
        + "  \"doglog_queue_capacity\": 4096,\n"
        + "  \"doglog_min_remaining_capacity\": "
        + minimumQueueRemaining
        + ",\n"
        + "  \"doglog_queue_acceptable\": "
        + (minimumQueueRemaining >= 1024 && !dogLogQueueFaultSeen)
        + ",\n"
        + "  \"doglog_queue_fault_seen\": "
        + dogLogQueueFaultSeen
        + ",\n"
        + "  \"heap_used_before_bytes\": "
        + heapBefore
        + ",\n"
        + "  \"heap_used_after_bytes\": "
        + heapAfter
        + ",\n"
        + "  \"behavior_sha256\": \""
        + HexFormat.of().formatHex(behaviorDigest)
        + "\"\n"
        + "}\n";
  }

  private static void runLoops(
      Robot robot,
      DIOSim towerSensor,
      IntegerSubscriber queueRemaining,
      AtomicLong minimumQueueRemaining,
      int loopCount,
      List<Long> durations,
      MessageDigest behaviorDigest) {
    long deadline = System.nanoTime();
    for (int loop = 0; loop < loopCount; loop++) {
      deadline += LOOP_PERIOD_NANOS;
      // Synchronous stepping waits for every vendor notifier; some Phoenix simulation notifiers
      // intentionally outlive this harness. Wall-clock pacing below provides the synchronization.
      SimHooks.stepTimingAsync(LOOP_PERIOD_SECONDS);
      double elapsedSeconds = loop * LOOP_PERIOD_SECONDS;
      robot.benchmarkSimulationStep(elapsedSeconds);
      towerSensor.setValue((loop / 50) % 2 == 0);
      DriverStationSim.notifyNewData();
      long start = System.nanoTime();
      robot.robotPeriodic();
      long duration = System.nanoTime() - start;
      if (durations != null) {
        durations.add(duration);
      }
      if (behaviorDigest != null) {
        behaviorDigest.update(robot.benchmarkBehaviorSnapshot().getBytes(UTF_8));
      }
      minimumQueueRemaining.accumulateAndGet(queueRemaining.get(), Math::min);
      long remaining = deadline - System.nanoTime();
      if (remaining > 0) {
        LockSupport.parkNanos(remaining);
      }
    }
  }

  private static long usedHeapBytes() {
    Runtime runtime = Runtime.getRuntime();
    return runtime.totalMemory() - runtime.freeMemory();
  }

  private LoopBenchmark() {}
}
