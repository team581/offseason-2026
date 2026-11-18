package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class InterpolatedTrajectoryRequestTest {
  private static void assertSpeeds(ChassisSpeeds speeds, double vx, double vy, double omega) {
    assertEquals(vx, speeds.vxMetersPerSecond, 1e-9);
    assertEquals(vy, speeds.vyMetersPerSecond, 1e-9);
    assertEquals(omega, speeds.omegaRadiansPerSecond, 1e-9);
  }

  @Test
  void publishedSpeedsAreIndependentOfCallerMutationAndCanBeCleared() {
    var request = new InterpolatedTrajectoryRequest();
    var start = new ChassisSpeeds(1, 0, 0);
    var end = new ChassisSpeeds(2, 0, 0);
    var output = new ChassisSpeeds();
    request.setStep(new VelocityStep(start, 5, 0.020), end);
    start.vxMetersPerSecond = 99;
    end.vxMetersPerSecond = 99;

    request.sample(5.010, output);
    assertSpeeds(output, 1.5, 0, 0);
    request.clear();
    request.sample(5.010, output);
    assertSpeeds(output, 0, 0, 0);
  }

  @Test
  void samplesOneTimedChassisRampAndHoldsItsEndpoint() {
    var request = new InterpolatedTrajectoryRequest();
    var start = new ChassisSpeeds(1, -2, 0.5);
    var end = new ChassisSpeeds(1.2, -1.6, -0.5);
    var output = new ChassisSpeeds();
    var step = new VelocityStep(start, 5, 0.020);
    request.setStep(step, end);

    request.sample(4.999, output);
    assertSpeeds(output, 1, -2, 0.5);
    request.sample(5.005, output);
    assertSpeeds(output, 1.05, -1.9, 0.25);

    // Republishing a cached follower result should not restart its interval.
    request.setStep(step, end);
    request.sample(5.015, output);
    assertSpeeds(output, 1.15, -1.7, -0.25);
    request.sample(5.030, output);
    assertSpeeds(output, 1.2, -1.6, -0.5);
  }
}
