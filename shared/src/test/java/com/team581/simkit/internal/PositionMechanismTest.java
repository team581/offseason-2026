package com.team581.simkit.internal;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

final class PositionMechanismTest {
  private static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(10.0, 35.0);

  @Test
  void boundsClampPositionAndStopAtBoundary() {
    var bounded =
        PositionMechanism.applyBounds(
            new TrapezoidProfile.State(2.0, 3.0), OptionalDouble.of(-1.0), OptionalDouble.of(1.0));

    assertThat(bounded.position).isEqualTo(1.0);
    assertThat(bounded.velocity).isEqualTo(0);
  }

  @Test
  void disabledStateHoldsPositionAndStopsVelocity() {
    var stopped = PositionMechanism.disabledState(new TrapezoidProfile.State(0.75, 4.0));

    assertThat(stopped.position).isEqualTo(0.75);
    assertThat(stopped.velocity).isEqualTo(0);
  }

  @Test
  void requestedGoalVelocityImprovesMotionTrackingProfile() {
    var current = new TrapezoidProfile.State(0.5, 2.0);
    var withVelocity =
        PositionMechanism.calculateProfile(
            CONSTRAINTS, 0.02, current, PositionMechanism.desiredMechanismState(0.54, 2.0));
    var withoutVelocity =
        PositionMechanism.calculateProfile(
            CONSTRAINTS, 0.02, current, PositionMechanism.desiredMechanismState(0.54, 0.0));

    assertThat(withVelocity.velocity).isGreaterThan(withoutVelocity.velocity);
  }

  @Test
  void stationaryGoalSettlesAtRequestedPosition() {
    var result =
        PositionMechanism.calculateProfile(
            CONSTRAINTS,
            1.0,
            new TrapezoidProfile.State(0.0, 0.0),
            PositionMechanism.desiredMechanismState(1.0, 0.0));

    assertThat(result.position).isCloseTo(1.0, offset(1e-9));
    assertThat(result.velocity).isCloseTo(0.0, offset(1e-9));
  }
}
