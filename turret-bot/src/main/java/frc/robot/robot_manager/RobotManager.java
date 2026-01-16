package frc.robot.robot_manager;

import com.team581.util.state_machines.StateMachineSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  public final Localization localization;
  public final Swerve swerve;
  public final Turret turret;
  public final Vision vision;

  public RobotManager(Localization localization, Swerve swerve, Turret turret, Vision vision) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.IDLE);
    this.turret = turret;
    this.localization = localization;
    this.swerve = swerve;
    this.vision = vision;
  }

  @Override
  protected RobotState getNextState(RobotState currentState) {
    return switch (currentState) {
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case AUTO_AIM -> turret.hubAimRequest();
      case MANUAL_AIM -> turret.manualAimRequest();
      case IDLE -> turret.idleRequest();
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    vision.addTurretObservation(
        Timer.getFPGATimestamp(), Rotation2d.fromDegrees(turret.getAngle()));

    MechanismVisualizer.log(localization.getPose(), turret.getAngle());
  }
}
