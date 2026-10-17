package frc.robot.sim;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.testing.TestDriveRequest;

/** Real Phoenix module simulation and offseason constants; no full Robot construction needed. */
final class SwerveFixture implements AutoCloseable {
  final TunerSwerveDrivetrain drivetrain =
      new TunerSwerveDrivetrain(
          RobotTunerConstants.DrivetrainConstants,
          RobotTunerConstants.FrontLeft,
          RobotTunerConstants.FrontRight,
          RobotTunerConstants.BackLeft,
          RobotTunerConstants.BackRight);
  private final TestDriveRequest request = new TestDriveRequest();
  private final Notifier physics;
  private double previousTime = Utils.getCurrentTimeSeconds();

  SwerveFixture(double voltage, Pose2d start) {
    drivetrain.resetPose(start);
    physics =
        new Notifier(
            () -> {
              double now = Utils.getCurrentTimeSeconds();
              drivetrain.updateSimState(now - previousTime, voltage);
              previousTime = now;
            });
    physics.startPeriodic(0.005);
    // Let the asynchronous vendor odometry become valid before starting the measurement.
    double deadline = Timer.getFPGATimestamp() + 3.0;
    while (!drivetrain.isOdometryValid() && Timer.getFPGATimestamp() < deadline) {
      Timer.delay(0.020);
    }
    if (!drivetrain.isOdometryValid()) {
      close();
      throw new IllegalStateException("Phoenix odometry did not initialize");
    }
    drivetrain.resetPose(start);
  }

  @Override
  public void close() {
    drivetrain.setControl(new SwerveRequest.Idle());
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    physics.close();
    drivetrain.close();
  }

  void applyRequest(ChassisSpeeds speeds) {
    if (!DriverStation.isTestEnabled()) {
      speeds = new ChassisSpeeds();
    }
    drivetrain.setControl(
        request
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  Pose2d pose() {
    return drivetrain.getState().Pose;
  }

  ChassisSpeeds speeds() {
    var state = drivetrain.getState();
    return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
  }
}
