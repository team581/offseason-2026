package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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

  /** Observe the applied module targets without advancing the Trailblazer follower again. */
  public ChassisSpeeds commandedSpeeds() {
    var drivetrain = robotManager.swerve.drivetrain;
    var state = drivetrain.getState();
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        drivetrain.getKinematics().toChassisSpeeds(state.ModuleTargets), state.Pose.getRotation());
  }

  public TestManager manager() {
    return testManager;
  }

  public ChassisSpeeds measuredSpeeds() {
    return robotManager.swerve.getFieldRelativeSpeeds();
  }

  public void runUntil(BooleanSupplier finished, Runnable recordSample, double timeoutSeconds) {
    double deadline = Timer.getFPGATimestamp() + timeoutSeconds;
    while (!finished.getAsBoolean()) {
      if (Timer.getFPGATimestamp() >= deadline) {
        throw new AssertionError("Headless robot deadline exceeded");
      }
      if (Thread.currentThread().isInterrupted()) {
        throw new AssertionError("Headless robot interrupted");
      }
      tick();
      recordSample.run();
    }
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
        () -> {},
        5.0);
    robotManager.localization.resetPose(start);
    tick();
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
