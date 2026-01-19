package frc.robot.robot_manager;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretCalculator;
import frc.robot.util.april_tags.TagMap;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;

public class RobotManager extends StateMachineSubsystem<RobotState> {
  private static final IntegerSubscriber TAG_AIM_ID = DogLog.tunable("TagAimID", 9);
  private static final DoubleSubscriber TIME_OF_FLIGHT = DogLog.tunable("TimeOfFlight", 0.0);

  public final Localization localization;
  public final Swerve swerve;
  public final Turret turret;
  public final Vision vision;

  private Pose2d robotPose = Pose2d.kZero;

  /** The robot translation clamped to be within the alliance zone. */
  private Translation2d robotTranslationInAllianceZone = Translation2d.kZero;

  // TODO: once this robot has a shooter, this will decide if it can shoot
  private boolean inAllianceZone = true;

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
        if (turret.goalOutOfBounds()) {
          swerveTurretCompensationAngle =
              TurretCalculator.calculateSwerveTurretCompensationAngle(
                  turretHubGoalAngle, robotPose.getRotation());
          yield RobotState.HUB_AIM_ADJUSTING_SWERVE;
        }
        yield currentState;
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        if (MathUtil.isNear(
                swerveTurretCompensationAngle, robotPose.getRotation().getDegrees(), 5, -180, 180)
            || TurretCalculator.doesTurretHaveRoom(turret.getAngle())) {
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
        vision.setState(VisionState.HUB_TAGS);
        turret.hubAimRequest();
      }
      case TAG_AIM -> {
        vision.setState(VisionState.TAGS);
        turret.tagAimRequest();
      }
      case LOCK_FORWARD -> {
        turret.lockForwardRequest();
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
      }
      case IDLE -> {
        vision.setState(VisionState.TAGS);
        turret.idleRequest();
      }
    }
  }

  @Override
  protected void whileInState(RobotState state) {
    DogLog.log("RobotManager/SwerveCompAngle", swerveTurretCompensationAngle);

    switch (getState()) {
      case HUB_AIM -> {
        turret.setHubAimAngle(turretHubGoalAngle);
        // TODO: Why are we continuously setting a normal drive request? Can't we just do that once?
        swerve.normalDriveRequest();
      }
      case HUB_AIM_ADJUSTING_SWERVE -> {
        swerve.snapsDriveRequest(swerveTurretCompensationAngle);
        var goalPose =
            ShootOnTheMove.getVelocityCompensatedGoal(
                FieldUtil.getHubPose(), swerve.getFieldRelativeSpeeds(), TIME_OF_FLIGHT.get());
        var aimingAngle =
            TurretCalculator.calculateTurretAimingAngle(
                robotTranslationInAllianceZone, robotPose.getRotation(), goalPose);
        turret.setHubAimAngle(aimingAngle);
      }
      case TAG_AIM -> {
        var goalPose = TagMap.getTagPose((int) TAG_AIM_ID.get());
        if (goalPose != null) {
          var aimingAngle =
              TurretCalculator.calculateTurretAimingAngle(
                  robotTranslationInAllianceZone,
                  robotPose.getRotation(),
                  goalPose.getTranslation());
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

  @Override
  protected void collectInputs() {
    var allianceZone = FieldUtil.getAllianceZone();
    inAllianceZone = allianceZone.contains(robotPose.getTranslation());
    robotPose = localization.getPose();

    robotTranslationInAllianceZone =
        inAllianceZone
            ? robotPose.getTranslation()
            : allianceZone.nearest(robotPose.getTranslation());
    DogLog.log(
        "RobotManager/LegalPose",
        new Pose2d(robotTranslationInAllianceZone, robotPose.getRotation()));
    DogLog.log("RobotManager/InAllianceZone", inAllianceZone);

    var goalPose = FieldUtil.getHubPose();

    if (FeatureFlags.SHOOT_ON_THE_MOVE.getAsBoolean()) {
      goalPose =
          ShootOnTheMove.getVelocityCompensatedGoal(
              goalPose, swerve.getFieldRelativeSpeeds(), TIME_OF_FLIGHT.get());
    }

    turretHubGoalAngle =
        TurretCalculator.calculateTurretAimingAngle(
            robotTranslationInAllianceZone, robotPose.getRotation(), goalPose);
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
