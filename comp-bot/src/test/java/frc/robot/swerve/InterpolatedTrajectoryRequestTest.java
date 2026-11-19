package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.CompTrailblazer;
import org.junit.jupiter.api.Test;

final class InterpolatedTrajectoryRequestTest {
  @Test
  void callbackBufferPreservesOrderAndReportsOverflow() {
    var buffer = new DriveRequestSamples("timestamped_velocity_and_force");
    var zero = new ChassisSpeeds();
    var ff = new InterpolatedTrajectoryRequest().forces(zero, Rotation2d.kZero);
    for (int i = 0; i < 129; i++) buffer.capture(i, 42, zero, zero, 0, 0, 0, 0, ff);
    assertEquals(1, buffer.dropped());
    var rows = buffer.drain();
    assertEquals(128 * 20, rows.length);
    for (int i = 0; i < 128; i++) {
      assertEquals(i, rows[i * 20]);
      assertEquals(42, rows[i * 20 + 1]);
    }
    assertEquals(0, buffer.drain().length);
  }

  @Test
  void directCommandResetsTheRampEvenWithoutAnOdometrySample() {
    var request = new InterpolatedTrajectoryRequest();
    var output = new ChassisSpeeds();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(), 0.020, 5.0, 1), new ChassisSpeeds(0.2, 0, 0));
    request.sample(5.000, output, new ChassisSpeeds());

    var start = new ChassisSpeeds(1, -2, 0.5);
    var end = new ChassisSpeeds(1.2, -1.6, -0.5);
    request.setDirect(start);
    var step = new VelocityStep(start, 0.020, 5.010, 2);
    request.setStep(step, end);
    start.vxMetersPerSecond = 99;
    end.vxMetersPerSecond = 99;

    request.sample(5.010, output, new ChassisSpeeds());
    assertEquals(1.0, output.vxMetersPerSecond, 1e-9);
    request.sample(5.020, output, new ChassisSpeeds());
    assertEquals(1.1, output.vxMetersPerSecond, 1e-9);
    assertEquals(-1.8, output.vyMetersPerSecond, 1e-9);
    assertEquals(0, output.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void nextStepStartsAtTheSampledSpeedAndPreservesTheSlew() {
    var request = new InterpolatedTrajectoryRequest();
    var output = new ChassisSpeeds();
    var start = new ChassisSpeeds();
    var first = new ChassisSpeeds(0.4, 0, 0);
    var second = new ChassisSpeeds(0.6, 0, 0);
    request.setStep(new VelocityStep(start, 0.040, 5.0, 1), first);
    request.sample(5.000, output, new ChassisSpeeds());
    request.setStep(new VelocityStep(first, 0.020, 5.020, 2), second);
    request.sample(5.020, output, new ChassisSpeeds());
    assertEquals(0.2, output.vxMetersPerSecond, 1e-9);
    request.sample(5.040, output, new ChassisSpeeds());
    assertEquals(0.4, output.vxMetersPerSecond, 1e-9);

    request.setStep(new VelocityStep(second, 0.020, 5.060, 3), second);
    request.sample(5.060, output, new ChassisSpeeds());
    assertEquals(0.6, output.vxMetersPerSecond, 1e-9);
  }

  @Test
  void samplesLateWithoutRestartingAndHoldsAfterMissedPublication() {
    var r = new InterpolatedTrajectoryRequest();
    var v = new ChassisSpeeds();
    var a = new ChassisSpeeds();
    var end = new ChassisSpeeds(-0.2, 0, 0);
    var step = new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.040, 10, 6);
    r.setStep(step, end);
    r.sample(10.010, v, a);
    assertEquals(0.1, v.vxMetersPerSecond, 1e-9);
    assertEquals(-10, a.vxMetersPerSecond, 1e-9);
    r.setStep(step, end);
    r.sample(10.030, v, a);
    assertEquals(-0.1, v.vxMetersPerSecond, 1e-9);
    r.sample(10.040, v, a);
    assertEquals(-0.2, v.vxMetersPerSecond, 1e-9);
    assertEquals(0, a.vxMetersPerSecond);
    r.sample(10.5, v, a);
    assertEquals(-0.2, v.vxMetersPerSecond);
    assertEquals(0, a.vxMetersPerSecond);
    r.setDirect(new ChassisSpeeds());
    r.sample(10.501, v, a);
    assertEquals(0, v.vxMetersPerSecond);
    assertEquals(0, a.vxMetersPerSecond);
  }

  @Test
  void stagedAndFirstEnabledCommandsHoldTheLaunchBeforeInterpolation() {
    var request = new InterpolatedTrajectoryRequest();
    var launch = new ChassisSpeeds(0.2, 0, 0);
    var output = new ChassisSpeeds();

    request.setDirect(launch);
    request.sample(5.000, output, new ChassisSpeeds());
    assertEquals(0.2, output.vxMetersPerSecond, 1e-9);
    request.setDirect(launch);
    request.sample(5.020, output, new ChassisSpeeds());
    assertEquals(0.2, output.vxMetersPerSecond, 1e-9);

    request.setStep(new VelocityStep(launch, 0.020, 5.043, 1), new ChassisSpeeds(0.4, 0, 0));
    request.sample(5.043, output, new ChassisSpeeds());
    assertEquals(0.2, output.vxMetersPerSecond, 1e-9);
    request.sample(5.053, output, new ChassisSpeeds());
    assertEquals(0.3, output.vxMetersPerSecond, 1e-9);
  }

  @Test
  void tinyYawCorrectionCannotThrottleTranslation() {
    var r = new InterpolatedTrajectoryRequest();
    var first = new ChassisSpeeds(0.20, 0, 0.02);
    r.setStep(new VelocityStep(new ChassisSpeeds(), 0.02, 1.0, 1), first);
    r.setStep(new VelocityStep(first, 0.02, 1.01, 2), new ChassisSpeeds(0.40, 0, 0.020001));
    var v = new ChassisSpeeds();
    var a = new ChassisSpeeds();
    r.sample(1.02, v, a);
    assertEquals(0.20, v.vxMetersPerSecond, 1e-8);
    assertEquals(10, a.vxMetersPerSecond, 1e-8);
    r.sample(1.041, v, a);
    assertEquals(0.020001, v.omegaRadiansPerSecond, 1e-9);
    assertEquals(0, a.omegaRadiansPerSecond);
  }

  @Test
  void wheelForcesProduceTheRequestedNetForceAndYawMoment() {
    var request = new InterpolatedTrajectoryRequest();
    var spec = CompTrailblazer.DRIVE_SPEC;
    for (double alpha : new double[] {0, -2.5}) {
      for (var heading : new Rotation2d[] {Rotation2d.kZero, Rotation2d.kCCW_90deg}) {
        var a = new ChassisSpeeds(3.2, 0, alpha);
        var ff = request.forces(a, heading);
        double fx = 0, fy = 0, torque = 0;
        for (int i = 0; i < 4; i++) {
          var location = spec.modulePositionsMeters().get(i);
          fx += ff.x_newtons[i];
          fy += ff.y_newtons[i];
          torque += location.getX() * ff.y_newtons[i] - location.getY() * ff.x_newtons[i];
        }
        assertEquals(spec.massKg() * 3.2 * heading.getCos(), fx, 1e-9);
        assertEquals(-spec.massKg() * 3.2 * heading.getSin(), fy, 1e-9);
        assertEquals(spec.yawMoiKgMetersSq() * alpha, torque, 1e-9);
      }
    }
  }
}
