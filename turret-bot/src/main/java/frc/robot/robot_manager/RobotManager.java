package frc.robot.robot_manager;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretCalculator;
import frc.robot.util.april_tags.TagMap;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private static final int TAG_AIM_ID = 15;
  private static final DoubleSubscriber TIME_OF_FLIGHT = DogLog.tunable("TimeOfFlight", 0.0);

  public final Localization localization;
  public final Swerve swerve;
  public final Turret turret;
  public final Vision vision;

  private Pose2d robotPose = Pose2d.kZero;
  private double swerveTurretCompensationAngle = 0.0;
  private double turretHubGoalAngle = 0.0;

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
      case HUB_AIM -> {
        // if (turret.goalOutOfBounds()) {
        //   swerveTurretCompensationAngle =
        // TurretCalculator.calculateSwerveTurretCompensationAngle(wantedTurretHubAngle, robotPose);
        //   yield RobotState.HUB_AIM_ADJUSTING_SWERVE;
        // }
        yield currentState;
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        if (MathUtil.isNear(
            swerveTurretCompensationAngle, robotPose.getRotation().getDegrees(), 5, -180, 180)) {
          yield RobotState.HUB_AIM;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case HUB_AIM, HUB_AIM_ADJUSTING_SWERVE -> {
        updateTuretHubGoalAngle();
        turret.hubAimRequest();
      }
      case TAG_AIM -> turret.tagAimRequest();
      case LOCK_FORWARD -> turret.lockForwardRequest();
      case IDLE -> turret.idleRequest();
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("RobotManager/SwerveCompAngle", swerveTurretCompensationAngle);

    vision.addTurretObservation(
        Timer.getFPGATimestamp(), Rotation2d.fromDegrees(turret.getAngle()));

    robotPose = localization.getPose();
    switch (getState()) {
      case HUB_AIM -> {
        updateTuretHubGoalAngle();
        turret.setHubAimAngle(turretHubGoalAngle);
        swerve.normalDriveRequest();
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        swerve.snapsDriveRequest(swerveTurretCompensationAngle);
        var goalPose =
            ShootOnTheMove.getVelocityCompensatedGoal(
                FieldUtil.RED_HUB_POSE, swerve.getFieldRelativeSpeeds(), TIME_OF_FLIGHT.get());
        var aimingAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, goalPose);
        turret.setHubAimAngle(aimingAngle);
      }
      case TAG_AIM -> {
        var goalPose = TagMap.getTagPose(TAG_AIM_ID);
        if (goalPose != null) {
          var aimingAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, goalPose);
          turret.setTagAimAngle(aimingAngle);
          DogLog.clearFault("Tag Aim: No valid ID");
        } else {
          DogLog.logFault("Tag Aim: No valid ID");
          setStateFromRequest(RobotState.IDLE);
        }
      }
      default -> {}
    }

    MechanismVisualizer.log(localization.getPose(), turret.getAngle());
  }

  private void updateTuretHubGoalAngle() {
    var goalPose =
        ShootOnTheMove.getVelocityCompensatedGoal(
            FieldUtil.RED_HUB_POSE, swerve.getFieldRelativeSpeeds(), TIME_OF_FLIGHT.get());
    turretHubGoalAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, goalPose);
  }

  public void hubAimRequest() {
    if (getState() != RobotState.HUB_AIM_ADJUSTING_SWERVE) {
      setStateFromRequest(RobotState.HUB_AIM);
    }
  }

  public void tagAimRequest() {
    setStateFromRequest(RobotState.TAG_AIM);
  }

  public void lockForwardRequest() {
    setStateFromRequest(RobotState.LOCK_FORWARD);
  }
}
