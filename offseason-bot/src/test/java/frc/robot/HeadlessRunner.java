package frc.robot;

import com.team581.simkit.SimKit;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.deploy.Deploy;
import frc.robot.health.HealthManager;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.testing.TestManager;
import frc.robot.testing.TestSelection;
import java.util.concurrent.locks.LockSupport;
import java.util.function.BooleanSupplier;

/** Runs the regular robot lifecycle, scheduler, and subsystem simulations without a GUI. */
public final class HeadlessRunner extends Robot {
  public HeadlessRunner() {
    robotInit();
    simulationInit();
  }

  @Override
  public void close() {
    DriverStationSim.setEnabled(false);
    tick();
    super.close();
  }

  public Deploy deploy() {
    return robotManager.hopperManager.deploy;
  }

  public HealthManager health() {
    return health;
  }

  public TestManager manager() {
    return testManager;
  }

  public void runUntil(BooleanSupplier finished, double timeoutSeconds) {
    double deadline = Timer.getFPGATimestamp() + timeoutSeconds;
    while (!finished.getAsBoolean()) {
      if (Timer.getFPGATimestamp() >= deadline) {
        throw new AssertionError("Headless robot deadline exceeded");
      }
      if (Thread.currentThread().isInterrupted()) {
        throw new AssertionError("Headless robot interrupted");
      }
      tick();
    }
  }

  /** Inject a collision through motor simulation, leaving deploy inputs/state to the scheduler. */
  public void seedDeployAngle(double angleDegrees) {
    SimKit.positionMechanism(
            "Deploy",
            builder -> {
              throw new IllegalStateException(
                  "Tick the robot to initialize deploy simulation first");
            })
        .seedPosition(-angleDegrees / 360.0);
    // Let Phoenix's simulated device publish the injected encoder values before input collection.
    LockSupport.parkNanos(20_000_000L);
  }

  public void select(TestSelection selection, Pose2d start) {
    DriverStationSim.setEnabled(false);
    NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable("Tests/SelectedTest")
        .getEntry("selected")
        .setString(selection.name());
    runUntil(
        () ->
            testManager.getState() == selection && robotManager.swerve.drivetrain.isOdometryValid(),
        5.0);
    robotManager.localization.resetPose(start);
    tick();
  }

  public ShooterHood shooterHood() {
    return shooterHood;
  }

  public void tick() {
    long loopStart = System.nanoTime();
    DriverStationSim.notifyNewData();
    loopFunc();
    long remaining = 20_000_000L - (System.nanoTime() - loopStart);
    if (remaining > 0) {
      LockSupport.parkNanos(remaining);
    }
  }
}
