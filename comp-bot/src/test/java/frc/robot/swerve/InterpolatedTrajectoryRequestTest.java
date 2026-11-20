package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.team581.trailblazer3.model.TickResult.VelocityStep;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.config.CompTrailblazer;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

final class InterpolatedTrajectoryRequestTest {
  private record Sample(ChassisSpeeds v, ChassisSpeeds a) {}

  private static final double TOLERANCE = 1e-9;

  private static void assertWrench(
      Feedforwards forces, double expectedFx, double expectedFy, double expectedTorque) {
    var positions = CompTrailblazer.DRIVE_SPEC.modulePositionsMeters();
    double fx = 0;
    double fy = 0;
    double torque = 0;
    for (int i = 0; i < positions.size(); i++) {
      var location = positions.get(i);
      fx += forces.x_newtons[i];
      fy += forces.y_newtons[i];
      torque += location.getX() * forces.y_newtons[i] - location.getY() * forces.x_newtons[i];
    }
    assertEquals(expectedFx, fx, TOLERANCE);
    assertEquals(expectedFy, fy, TOLERANCE);
    assertEquals(expectedTorque, torque, TOLERANCE);
  }

  private static Sample sampleAt(InterpolatedTrajectoryRequest request, double time) {
    var v = new ChassisSpeeds();
    var a = new ChassisSpeeds();
    request.sample(time, v, a);
    return new Sample(v, a);
  }

  @Test
  void adoptsReseededVelocityWithoutLongRamp() {
    var request = new InterpolatedTrajectoryRequest();
    var held = new ChassisSpeeds(0.502, 0, 0);
    request.setStep(
        new VelocityStep(new ChassisSpeeds(1.5, 0, 0), 0.020, 5.000, 1),
        new ChassisSpeeds(1.6, 0, 0));
    request.setStep(new VelocityStep(new ChassisSpeeds(0.50, 0, 0), 0.020, 5.020, 2), held);
    request.setStep(new VelocityStep(held, 0.020, 5.040, 3), held);

    var sample = sampleAt(request, 5.050);
    assertEquals(0.502, sample.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, sample.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void angularBrakingTowardZeroClampsGraceAtZero() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0, 0, 0.4), 0.020, 2.0, 1),
        new ChassisSpeeds(0, 0, 0.08));

    var atZero = sampleAt(request, 2.025);
    assertEquals(0.0, atZero.v().omegaRadiansPerSecond, TOLERANCE);
    assertEquals(0.0, atZero.a().omegaRadiansPerSecond, TOLERANCE);

    var afterZero = sampleAt(request, 2.030);
    assertEquals(0.0, afterZero.v().omegaRadiansPerSecond, TOLERANCE);
    assertEquals(0.0, afterZero.a().omegaRadiansPerSecond, TOLERANCE);
  }

  @Test
  void brakingTowardZeroClampsGraceAtZero() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.4, 0, 0), 0.020, 2.0, 1),
        new ChassisSpeeds(0.08, 0, 0));

    var beforeZero = sampleAt(request, 2.024);
    assertEquals(0.016, beforeZero.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-16.0, beforeZero.a().vxMetersPerSecond, TOLERANCE);

    var atZero = sampleAt(request, 2.025);
    assertEquals(0.0, atZero.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, atZero.a().vxMetersPerSecond, TOLERANCE);

    var afterZero = sampleAt(request, 2.030);
    assertEquals(0.0, afterZero.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, afterZero.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void clampsEarlyPublishTimeToStepStart() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.020, 5.040, 1),
        new ChassisSpeeds(0.4, 0, 0),
        5.035);

    var atStart = sampleAt(request, 5.040);
    assertEquals(0.2, atStart.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(10.0, atStart.a().vxMetersPerSecond, TOLERANCE);

    var midpoint = sampleAt(request, 5.050);
    assertEquals(0.3, midpoint.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(10.0, midpoint.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void crossingZeroIntoTinyOppositeSpeedDoesNotKeepReversing() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.020, 2.0, 1),
        new ChassisSpeeds(-0.01, 0, 0));

    var afterEnd = sampleAt(request, 2.024);
    assertEquals(-0.01, afterEnd.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, afterEnd.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void directCommandAppliesSpeedWithZeroAcceleration() {
    var request = new InterpolatedTrajectoryRequest();
    request.setDirect(new ChassisSpeeds(0.2, 0, 0));

    var held = sampleAt(request, 5.000);
    assertEquals(0.2, held.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, held.a().vxMetersPerSecond, TOLERANCE);

    request.setDirect(new ChassisSpeeds());
    var stopped = sampleAt(request, 5.020);
    assertEquals(0.0, stopped.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, stopped.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void directCommandClearsPriorRampForSubsequentStep() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(), 0.020, 5.000, 1), new ChassisSpeeds(0.2, 0, 0));
    request.setDirect(new ChassisSpeeds(1.0, 0, 0));
    request.setStep(
        new VelocityStep(new ChassisSpeeds(1.0, 0, 0), 0.020, 5.010, 2),
        new ChassisSpeeds(1.2, 0, 0));

    assertEquals(1.0, sampleAt(request, 5.010).v().vxMetersPerSecond, TOLERANCE);
    assertEquals(1.1, sampleAt(request, 5.020).v().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void holdingStepAfterJitterOverrunSettlesWithoutOvershooting() {
    var request = new InterpolatedTrajectoryRequest();
    var target = new ChassisSpeeds(0.4, 0, 0);
    request.setStep(new VelocityStep(new ChassisSpeeds(), 0.020, 1.000, 1), target);
    request.setStep(new VelocityStep(target, 0.020, 1.020, 2), target, 1.024);

    var atPublish = sampleAt(request, 1.024);
    assertEquals(0.48, atPublish.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-20.0, atPublish.a().vxMetersPerSecond, TOLERANCE);

    var atSettled = sampleAt(request, 1.028);
    assertEquals(0.40, atSettled.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, atSettled.a().vxMetersPerSecond, TOLERANCE);

    var afterSettled = sampleAt(request, 1.034);
    assertEquals(0.40, afterSettled.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, afterSettled.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void holdsContinuouslyWithZeroAccelerationAfterGraceExpires() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.040, 10.0, 1),
        new ChassisSpeeds(-0.2, 0, 0));

    var atPlannedEnd = sampleAt(request, 10.040);
    assertEquals(-0.2, atPlannedEnd.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-10.0, atPlannedEnd.a().vxMetersPerSecond, TOLERANCE);

    var atGraceEnd = sampleAt(request, 10.048);
    assertEquals(-0.28, atGraceEnd.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, atGraceEnd.a().vxMetersPerSecond, TOLERANCE);

    var longAfter = sampleAt(request, 10.500);
    assertEquals(-0.28, longAfter.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, longAfter.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void ignoresCallerMutationAfterPublishingStep() {
    var request = new InterpolatedTrajectoryRequest();
    var start = new ChassisSpeeds(1.0, -2.0, 0.5);
    var end = new ChassisSpeeds(1.2, -1.6, -0.5);
    request.setStep(new VelocityStep(start, 0.020, 5.010, 1), end);
    start.vxMetersPerSecond = 99;
    end.vxMetersPerSecond = 99;

    var midpoint = sampleAt(request, 5.020);
    assertEquals(1.1, midpoint.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-1.8, midpoint.v().vyMetersPerSecond, TOLERANCE);
    assertEquals(0.0, midpoint.v().omegaRadiansPerSecond, TOLERANCE);
  }

  @Test
  void ignoresDuplicateAndOlderSequencesEvenAfterDirectCommand() {
    var request = new InterpolatedTrajectoryRequest();
    var step = new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.040, 10.0, 6);
    var end = new ChassisSpeeds(-0.2, 0, 0);
    request.setStep(step, end);
    request.setStep(step, end, 10.010);

    var afterDuplicate = sampleAt(request, 10.030);
    assertEquals(-0.1, afterDuplicate.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-10.0, afterDuplicate.a().vxMetersPerSecond, TOLERANCE);

    request.setDirect(new ChassisSpeeds());
    request.setStep(step, end, 10.040);
    request.setStep(
        new VelocityStep(new ChassisSpeeds(1.0, 0, 0), 0.020, 10.050, 5),
        new ChassisSpeeds(2.0, 0, 0));

    var afterStale = sampleAt(request, 10.055);
    assertEquals(0.0, afterStale.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, afterStale.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void keepsAcceleratingAcrossBriefLoopJitter() {
    var request = new InterpolatedTrajectoryRequest();
    var first = new ChassisSpeeds(0.17, 0, 0);
    request.setStep(new VelocityStep(new ChassisSpeeds(), 0.020, 1.000, 1), first, 1.002);

    var duringGrace = sampleAt(request, 1.024);
    assertEquals(0.187, duringGrace.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(8.5, duringGrace.a().vxMetersPerSecond, TOLERANCE);

    request.setStep(
        new VelocityStep(first, 0.024, 1.024, 2), new ChassisSpeeds(0.374, 0, 0), 1.026);
    var atNextPublish = sampleAt(request, 1.026);
    assertEquals(0.204, atNextPublish.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(8.5, atNextPublish.a().vxMetersPerSecond, TOLERANCE);

    var midNext = sampleAt(request, 1.036);
    assertEquals(0.289, midNext.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(8.5, midNext.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void nextStepContinuesFromSampledSpeedAndPreservesSlew() {
    var request = new InterpolatedTrajectoryRequest();
    var first = new ChassisSpeeds(0.4, 0, 0);
    var second = new ChassisSpeeds(0.6, 0, 0);
    request.setStep(new VelocityStep(new ChassisSpeeds(), 0.040, 5.000, 1), first);
    request.setStep(new VelocityStep(first, 0.020, 5.020, 2), second);

    assertEquals(0.2, sampleAt(request, 5.020).v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.4, sampleAt(request, 5.040).v().vxMetersPerSecond, TOLERANCE);

    request.setStep(new VelocityStep(second, 0.020, 5.060, 3), second);
    assertEquals(0.6, sampleAt(request, 5.060).v().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void nonPositiveStepDurationAppliesEndpointDirectly() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(1.0, 0, 0), 0.0, 5.000, 1),
        new ChassisSpeeds(0.4, 0, 0));

    var sample = sampleAt(request, 5.000);
    assertEquals(0.4, sample.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, sample.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void reversalAfterJitterOverrunStaysWithinCommandedRate() {
    var request = new InterpolatedTrajectoryRequest();
    var peak = new ChassisSpeeds(0.4, 0, 0);
    request.setStep(new VelocityStep(new ChassisSpeeds(), 0.020, 1.000, 1), peak);
    request.setStep(new VelocityStep(peak, 0.020, 1.020, 2), new ChassisSpeeds(), 1.024);

    var atPublish = sampleAt(request, 1.024);
    assertEquals(0.48, atPublish.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(-20.0, atPublish.a().vxMetersPerSecond, TOLERANCE);

    var atStop = sampleAt(request, 1.048);
    assertEquals(0.0, atStop.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, atStop.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void samplesBeforeStartTimeHoldStartSpeedWithoutAcceleration() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.2, 0, 0), 0.020, 5.000, 1),
        new ChassisSpeeds(0.4, 0, 0));

    var beforeStart = sampleAt(request, 4.995);
    assertEquals(0.2, beforeStart.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, beforeStart.a().vxMetersPerSecond, TOLERANCE);

    var midpoint = sampleAt(request, 5.010);
    assertEquals(0.3, midpoint.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(10.0, midpoint.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void stopsAtZeroWithoutReversing() {
    var request = new InterpolatedTrajectoryRequest();
    request.setStep(
        new VelocityStep(new ChassisSpeeds(0.12, 0, 0), 0.020, 2.0, 1), new ChassisSpeeds());

    var afterEnd = sampleAt(request, 2.024);
    assertEquals(0.0, afterEnd.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(0.0, afterEnd.a().vxMetersPerSecond, TOLERANCE);
  }

  @Test
  void tinyYawCorrectionCannotThrottleTranslation() {
    var request = new InterpolatedTrajectoryRequest();
    var first = new ChassisSpeeds(0.20, 0, 0.02);
    request.setStep(new VelocityStep(new ChassisSpeeds(), 0.020, 1.000, 1), first);
    request.setStep(new VelocityStep(first, 0.020, 1.010, 2), new ChassisSpeeds(0.40, 0, 0.020001));

    var duringTranslation = sampleAt(request, 1.020);
    assertEquals(0.20, duringTranslation.v().vxMetersPerSecond, TOLERANCE);
    assertEquals(10.0, duringTranslation.a().vxMetersPerSecond, TOLERANCE);

    var atYawEnd = sampleAt(request, 1.040);
    assertEquals(0.020001, atYawEnd.v().omegaRadiansPerSecond, TOLERANCE);

    var afterGrace = sampleAt(request, 1.050);
    assertEquals(0.0, afterGrace.a().omegaRadiansPerSecond, TOLERANCE);
  }

  @CsvSource({"-2.5, 0.0", "-2.5, 90.0", "0.0, 0.0", "0.0, 90.0", "3.0, 37.5"})
  @ParameterizedTest
  void wheelForcesProduceTheRequestedNetForceAndYawMoment(double alpha, double headingDeg) {
    var request = new InterpolatedTrajectoryRequest();
    var spec = CompTrailblazer.DRIVE_SPEC;
    var heading = Rotation2d.fromDegrees(headingDeg);
    var forces = request.forces(new ChassisSpeeds(3.2, -1.4, alpha), heading);

    assertWrench(
        forces,
        spec.massKg() * (3.2 * heading.getCos() - 1.4 * heading.getSin()),
        spec.massKg() * (-3.2 * heading.getSin() - 1.4 * heading.getCos()),
        spec.yawMoiKgMetersSq() * alpha);
  }
}
