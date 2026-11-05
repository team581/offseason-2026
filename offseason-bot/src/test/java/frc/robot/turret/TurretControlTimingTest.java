package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.scheduling.SubsystemExecutionSequencer;
import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;
import org.junit.jupiter.api.Test;

final class TurretControlTimingTest {
  private enum State {
    ACTIVE
  }

  @Test
  void freshInputsReachMotorInSameSchedulerPassAndStuckClearsVelocity() {
    HAL.initialize(500, 0);
    var motor = new TalonFX(58);
    var turret = new Turret(motor);
    turret.setState(TurretState.SCORE);
    double[] sampledTarget = {-180.0, 0.0};
    double[] nextTarget = {-160.0, Math.PI};
    boolean[] stuck = {false};

    // Register the producer before its input source: priorities must enforce the dependency.
    new StateMachineSubsystem<State>(SubsystemPriority.ROBOT_MANAGER, State.ACTIVE) {
      private double angle;
      private double velocity;

      @Override
      protected void collectInputs() {
        angle = sampledTarget[0];
        velocity = sampledTarget[1];
      }

      @Override
      protected void whileInState(State state) {
        if (stuck[0]) {
          turret.stuckRequest();
        } else {
          turret.scoreRequest(angle, velocity);
        }
      }
    };
    new StateMachineSubsystem<State>(SubsystemPriority.SWERVE, State.ACTIVE) {
      @Override
      protected void collectInputs() {
        System.arraycopy(nextTarget, 0, sampledTarget, 0, 2);
      }
    };

    for (double[] target : new double[][] {{-160.0, Math.PI}, {-200.0, -2.0}}) {
      System.arraycopy(target, 0, nextTarget, 0, 2);
      SubsystemExecutionSequencer.periodic();
      var request = (PositionVoltage) motor.getAppliedControl();
      assertThat(request.Position).isCloseTo(Units.degreesToRotations(target[0]), within(1e-9));
      assertThat(request.Velocity).isCloseTo(target[1] / (2.0 * Math.PI), within(1e-9));
      assertThat(request.UpdateFreqHz).isZero();
    }

    stuck[0] = true;
    SubsystemExecutionSequencer.periodic();
    var request = (PositionVoltage) motor.getAppliedControl();
    assertThat(request.Position)
        .isEqualTo(Units.degreesToRotations(TurretConfig.FAUX_DUMPER_ANGLE));
    assertThat(request.Velocity).isZero();
  }
}
