package frc.robot.shooter.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.mechanisms.PowerManaged;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> implements PowerManaged {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  public Feeder(TalonFX topMotor, TalonFX bottomMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    topMotor.getConfigurator().apply(FeederConfig.TOP_MOTOR_CONFIG);
    bottomMotor.getConfigurator().apply(FeederConfig.BOTTOM_MOTOR_CONFIG);
    this.topMotor = topMotor;
    this.bottomMotor = bottomMotor;
  }

  public void ballFillingRequest() {
    setStateFromRequest(FeederState.BALL_FILLING);
  }

  public void ejectRequest() {
    setStateFromRequest(FeederState.EJECT);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }

  public void intakeRequest() {
    setStateFromRequest(FeederState.INTAKING);
  }

  public void shootRequest() {
    setStateFromRequest(FeederState.SHOOTING);
  }
}
