package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.math.CircularFilter;
import com.team581.math.MathHelpers;
import com.team581.mechanisms.PowerManaged;
import com.team581.swerve.DriveSource;
import com.team581.swerve.DriveSourceType;
import com.team581.swerve.SwerveAssist;
import com.team581.swerve.TrailblazerDriveSource;
import com.team581.swerve.XboxControllerDriveSource;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.config.DSOptions;
import frc.robot.config.FeatureFlags;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.generated.RobotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

@SuppressWarnings("unused")
public class Swerve extends StateMachineSubsystem<SwerveState> implements PowerManaged {

  public static final double TRANSLATION_STD_DEV = 0.01;

  public static final double MAX_LINEAR_RATE = 4.75;

  // TODO(simonstoryparker): make separate ones for scoring
  private static final DoubleSubscriber MAX_LINEAR_RATE_SCORING =
      DogLog.tunable("Swerve/MaxLinearRateScoring", 4.0);

  // TODO(simonstoryparker): make separate ones for scoring
  private static final DoubleSubscriber MAX_LINEAR_RATE_FEEDING =
      DogLog.tunable("Swerve/MaxLinearRateFeeding", 4.0);

  private static final DoubleSubscriber MAX_ANGULAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxAngularRateShootingRot", 4.0);

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4.0);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);
  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  public static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(15, 0, 0);

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(1);

  private final SlewRateLimiter maxLinearVelocityRateLimiter =
      new SlewRateLimiter(100.0, -10.0, MAX_LINEAR_RATE);

  private final SlewRateLimiter maxAngularVelocityRateLimiter = new SlewRateLimiter(5.0);

  public final TunerSwerveDrivetrain drivetrain;
  private final XboxControllerDriveSource teleopDriveSource;
  private final TrailblazerDriveSource trailblazerDriveSource;

  private DriveSource driveSource;

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}. */
  private final SwerveRequest.FieldCentric driverPerspective =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.0)
          .withRotationalDeadband(0.0);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveSnaps =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.0)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}. */
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(0.07)
          .withRotationalDeadband(0.05)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final HealthManager health;

  private double lastSimTime;

  private @Nullable Notifier simNotifier = null;
  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private double currentMaxLinearRate = MAX_LINEAR_RATE;
  private double currentMaxAngularRate = TELEOP_MAX_ANGULAR_RATE.getRotations();

  private Rotation2d currentMaxAngularRateRotation = TELEOP_MAX_ANGULAR_RATE;

  private boolean ableToBumpAssist = false;
  private ChassisSpeeds motionCommand = new ChassisSpeeds();
  private Translation2d motionCenterOfRotation = Translation2d.kZero;

  public Swerve(
      TunerSwerveDrivetrain drivetrain,
      HealthManager health,
      XboxController driverController,
      Trailblazer trailblazer) {
    super(SubsystemPriority.SWERVE, SwerveState.MANUAL);
    this.drivetrain = drivetrain;
    this.health = health;

    if (Utils.isSimulation()) {
      startSimThread();
    }

    drivetrain.setStateStdDevs(
        new Matrix<>(VecBuilder.fill(TRANSLATION_STD_DEV, TRANSLATION_STD_DEV, 0.002)));

    this.teleopDriveSource =
        new XboxControllerDriveSource(
            driverController, Swerve.MAX_LINEAR_RATE, Swerve.TELEOP_MAX_ANGULAR_RATE);
    drivePerspectiveSnaps.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    this.trailblazerDriveSource =
        new TrailblazerDriveSource(
            trailblazer, () -> drivetrainState.Pose, this::getFieldRelativeSpeeds);
    driveSource = teleopDriveSource;
  }

  @Override
  public void applyCurrentLimits(double supplyCurrentLimit) {
    drivetrain
        .getModule(0)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            RobotTunerConstants.FrontLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(1)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            RobotTunerConstants.FrontRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(2)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            RobotTunerConstants.BackLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));

    drivetrain
        .getModule(3)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            RobotTunerConstants.BackRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
  }

  public void climbAssistDriveRequest() {
    setStateFromRequest(SwerveState.CLIMB_ASSIST);
  }

  public boolean driverStillDecidingSotm() {
    return false;
  }

  public boolean driverWantsSotm() {
    return true;
  }

  public void feedRequest() {
    setStateFromRequest(SwerveState.FEED);
  }

  public SwerveDriveState getDriveState() {
    return drivetrainState;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public boolean isMovingBeyondSafeSpeed() {
    var currentSpeeds = getFieldRelativeSpeeds();

    return MathHelpers.getLinearVelocity(currentSpeeds) > currentMaxLinearRate;
  }

  public boolean isNear(double angle, double tolerance) {
    return MathUtil.isNear(
        drivetrainState.Pose.getRotation().getDegrees(), angle, tolerance, -180.0, 180.0);
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  /** Freeze one command after autos runs and before RobotManager calculates the shot. */
  public ChassisSpeeds prepareMotionCommand() {
    var source = getState() == SwerveState.CLIMB_ASSIST ? trailblazerDriveSource : driveSource;
    var requested = source.getRequestedSpeeds();
    var translation = new Translation2d(requested.vxMetersPerSecond, requested.vyMetersPerSecond);
    if (source.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {
      translation =
          translation.rotateBy(FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);
    }
    if (translation.getNorm() < 0.07) {
      translation = Translation2d.kZero;
    }
    double omega =
        MathUtil.isNear(0, requested.omegaRadiansPerSecond, 0.05)
            ? 0.0
            : requested.omegaRadiansPerSecond;
    boolean usingBumpAssist =
        ableToBumpAssist
            && source.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP;
    motionCenterOfRotation =
        usingBumpAssist ? drivePerspectiveSnaps.CenterOfRotation : Translation2d.kZero;
    if (usingBumpAssist) {
      var target =
          SwerveAssist.getRoundedSnapAngle(
              drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE);
      omega =
          MathUtil.clamp(
              drivePerspectiveSnaps.HeadingController.calculate(
                  drivetrainState.Pose.getRotation().getRadians(),
                  target.getRadians(),
                  Utils.getCurrentTimeSeconds()),
              -Units.rotationsToRadians(currentMaxAngularRate),
              Units.rotationsToRadians(currentMaxAngularRate));
    }
    motionCommand =
        DriverStation.isEnabled()
            ? new ChassisSpeeds(translation.getX(), translation.getY(), omega)
            : new ChassisSpeeds();
    DogLog.log("Swerve/MotionCommandFieldRelative", motionCommand);
    // Phoenix interprets translation at the center of rotation; aiming needs chassis-center speed.
    var center = motionCenterOfRotation.rotateBy(drivetrainState.Pose.getRotation());
    return new ChassisSpeeds(
        motionCommand.vxMetersPerSecond + motionCommand.omegaRadiansPerSecond * center.getY(),
        motionCommand.vyMetersPerSecond - motionCommand.omegaRadiansPerSecond * center.getX(),
        motionCommand.omegaRadiansPerSecond);
  }

  public void scoreRequest() {
    setStateFromRequest(SwerveState.SCORE);
  }

  /**
   * Set the {@link DriveSource} to use. For some states, this might be ignored (ex. a state where
   * we are always using Trailblazer speeds instead of teleop).
   *
   * <p>This is meant to be called by the autos subsystem periodically, to ensure auto speeds are
   * being applied at the correct times.
   */
  public void setDriveSourceType(DriveSourceType type) {
    this.driveSource =
        switch (type) {
          case DRIVER_PERSPECTIVE_OPEN_LOOP -> teleopDriveSource;
          case FIELD_CENTRIC_CLOSED_LOOP -> trailblazerDriveSource;
        };
  }

  public void warmupFeedRequest() {
    setStateFromRequest(SwerveState.WARMUP_FEED);
  }

  public void warmupScoreRequest() {
    setStateFromRequest(SwerveState.WARMUP_SCORE);
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    var request =
        driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            ? driverPerspective
            : fieldCentric;
    request.withDriveRequestType(
        ableToBumpAssist
                || getState() == SwerveState.CLIMB_ASSIST
                || driveSource.getDriveSourceType() == DriveSourceType.FIELD_CENTRIC_CLOSED_LOOP
            ? DriveRequestType.Velocity
            : DriveRequestType.OpenLoopVoltage);
    drivetrain.setControl(
        request
            .withVelocityX(motionCommand.vxMetersPerSecond)
            .withVelocityY(motionCommand.vyMetersPerSecond)
            .withRotationalRate(motionCommand.omegaRadiansPerSecond)
            .withCenterOfRotation(motionCenterOfRotation));

    if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
      var current = drivetrainState.ModuleStates;
      var targets = drivetrainState.ModuleTargets;
      boolean isMisaligned = false;

      for (int i = 0; i < current.length; i++) {
        var actual = current[i];
        var target = targets[i];

        if (MathUtil.isNear(actual.angle.getDegrees(), target.angle.getDegrees(), 10, -180, 180)
            || MathUtil.isNear(
                actual.angle.getDegrees(), target.angle.getDegrees() + 180, 10, -180, 180)) {
          // it's within tolerance
        } else {
          isMisaligned = true;
          break;
        }
      }

      if (isMisaligned) {
        DogLog.logFault("Swerve modules not pointed straight", AlertType.kError);
      } else {
        DogLog.clearFault("Swerve modules not pointed straight");
      }
    } else {
      DogLog.clearFault("Swerve modules not pointed straight");
    }

    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/FieldRelativeSpeeds", fieldRelativeSpeeds);
    DogLog.log("Swerve/AbleToBumpAssist", ableToBumpAssist);

    DogLog.log("Swerve/DriverWantsSOTM", driverWantsSotm());
    DogLog.log("Swerve/DriverStillDecidingSotm", driverStillDecidingSotm());
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());

    ableToBumpAssist =
        DSOptions.USE_BUMP_ASSIST.get()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToBumpAssist(drivetrainState.Pose, fieldRelativeSpeeds);

    // Use the normal driving caps in teleop, including while scoring and feeding.
    var velocityLimitState = DriverStation.isTeleop() ? SwerveState.MANUAL : getState();
    switch (velocityLimitState) {
      case SCORE -> {
        currentMaxAngularRate =
            maxAngularVelocityRateLimiter.calculate(MAX_ANGULAR_RATE_SHOOTING.get());
        currentMaxLinearRate =
            maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE_SCORING.get());
      }
      case FEED -> {
        currentMaxAngularRate =
            maxAngularVelocityRateLimiter.calculate(MAX_ANGULAR_RATE_SHOOTING.get());
        currentMaxLinearRate =
            maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE_FEEDING.get());
      }
      default -> {
        currentMaxAngularRate =
            maxAngularVelocityRateLimiter.calculate(Units.radiansToRotations(MAX_ANGULAR_RATE));
        currentMaxLinearRate = maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE);
      }
    }

    currentMaxAngularRateRotation = Rotation2d.fromRotations(currentMaxAngularRate);
    teleopDriveSource.setMaxVelocity(currentMaxLinearRate, currentMaxAngularRateRotation);

    drivePerspectiveSnaps.withMaxAbsRotationalRate(Units.rotationsToRadians(currentMaxAngularRate));

    java.util.Optional<Rotation2d> targetRotation = java.util.Optional.empty();
    switch (getState()) {
      case MANUAL, WARMUP_SCORE, SCORE, WARMUP_FEED, FEED -> {
        if (ableToBumpAssist) {
          targetRotation =
              java.util.Optional.of(
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE));
        }
      }
      default -> {}
    }

    java.util.Optional<Translation2d> closestObstacle =
        FieldUtil.SNAPPING_OBSTACLES.closestPoint(drivetrainState.Pose.getTranslation());
    Translation2d centerOfRotation = Translation2d.kZero;
    if (FeatureFlags.DYNAMIC_CENTER_OF_ROTATION.getAsBoolean()
        && closestObstacle.isPresent()
        && targetRotation.isPresent()) {
      if (drivetrainState.Pose.getTranslation().getDistance(closestObstacle.orElseThrow())
          <= Units.feetToMeters(1.0) + 0.4) {
        if (MathUtil.isNear(
            0,
            drivetrainState.Pose.getRotation().minus(targetRotation.orElseThrow()).getDegrees(),
            50.0)) {
          centerOfRotation =
              closestObstacle
                  .orElseThrow()
                  .minus(drivetrainState.Pose.getTranslation())
                  .rotateBy(drivetrainState.Pose.getRotation().unaryMinus());
        }
      }
    }
    drivePerspectiveSnaps.withCenterOfRotation(centerOfRotation);
  }
}
