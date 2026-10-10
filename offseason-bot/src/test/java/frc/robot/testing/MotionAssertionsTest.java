package frc.robot.testing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class MotionAssertionsTest {
  @Test
  void directionChangeAtConstantSpeedStillCountsAsAcceleration() {
    var assertions = new MotionAssertions(1, 0.75);
    assertions.start(0, new ChassisSpeeds(0.2, 0, 0));
    assertThat(assertions.check(0.02, new ChassisSpeeds(0.2, 0, 0), new ChassisSpeeds(0, 0.2, 0)))
        .hasValue("Commanded vector acceleration exceeded limit");
  }

  @Test
  void measuredOverspeedAndSidewaysAccelerationFail() {
    var assertions = new MotionAssertions(1, 0.75);
    assertions.start(0, new ChassisSpeeds());
    assertThat(assertions.check(0.02, new ChassisSpeeds(1.11, 0, 0), new ChassisSpeeds()))
        .hasValue("Measured velocity exceeded tolerance");
    assertions.start(0, new ChassisSpeeds());
    assertThat(assertions.check(0.02, new ChassisSpeeds(0, 0.1, 0), new ChassisSpeeds()))
        .hasValue("Measured vector acceleration exceeded tolerance");
  }

  @Test
  void nonuniformSamplesUseInterpolatedWindowAndNormalBrakingPasses() {
    var assertions = new MotionAssertions(1, 0.75);
    assertions.start(0, new ChassisSpeeds());
    for (double t : new double[] {0.03, 0.07, 0.12, 0.17, 0.21}) {
      var speeds = new ChassisSpeeds(t * 0.5, 0, 0);
      assertThat(assertions.check(t, speeds, speeds)).isEmpty();
    }
    assertThat(assertions.peakMeasuredAcceleration()).isCloseTo(0.5, offset(1e-9));
    assertThat(
            assertions.check(0.25, new ChassisSpeeds(0.085, 0, 0), new ChassisSpeeds(0.085, 0, 0)))
        .isEmpty();
  }

  @Test
  void startupJumpFailsWithoutGracePeriod() {
    var assertions = new MotionAssertions(1, 0.75);
    assertions.start(0, new ChassisSpeeds());
    assertThat(assertions.check(0.02, new ChassisSpeeds(), new ChassisSpeeds(0.5, 0, 0)))
        .hasValue("Commanded vector acceleration exceeded limit");
    assertThat(assertions.peakCommandAcceleration()).isEqualTo(25);
  }

  @Test
  void timestampsAndEverySpeedComponentMustBeValid() {
    var assertions = new MotionAssertions(1, 0.75);
    assertions.start(1, new ChassisSpeeds());
    assertThat(assertions.check(1, new ChassisSpeeds(), new ChassisSpeeds()))
        .hasValue("Motion timestamps must increase");
    assertThat(assertions.check(1.11, new ChassisSpeeds(), new ChassisSpeeds()))
        .hasValue("Motion sample gap exceeded 100 ms");
    assertThat(assertions.check(1.02, new ChassisSpeeds(0, 0, Double.NaN), new ChassisSpeeds()))
        .hasValue("Nonfinite motion data");
  }
}
