package frc.robot.sim;

import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.concurrent.locks.LockSupport;
import java.util.function.BooleanSupplier;

/** A small real-time loop; the fixture owns physics and the routine owns test behavior. */
public final class HeadlessRunner {
  public static void run(
      StateMachine<?> routine,
      BooleanSupplier finished,
      Runnable recordSample,
      double timeoutSeconds) {
    double deadline = Timer.getFPGATimestamp() + timeoutSeconds;
    while (!finished.getAsBoolean()) {
      if (Timer.getFPGATimestamp() >= deadline) {
        throw new AssertionError("Headless runner deadline exceeded");
      }
      if (Thread.currentThread().isInterrupted()) {
        throw new AssertionError("Headless runner interrupted");
      }
      long loopStart = System.nanoTime();
      DriverStationSim.notifyNewData();
      routine.beforePeriodic();
      routine.periodic();
      recordSample.run();
      long remaining = 20_000_000L - (System.nanoTime() - loopStart);
      if (remaining > 0) {
        LockSupport.parkNanos(remaining);
      }
    }
  }

  private HeadlessRunner() {}
}
