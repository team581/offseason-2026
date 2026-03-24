package frc.robot.swerve;

import static edu.wpi.first.units.Units.Radians;

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
import com.team581.util.FmsUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.generated.CompTunerConstants;
import frc.robot.generated.CompTunerConstants.TunerSwerveDrivetrain;
import frc.robot.health.HealthManager;
import frc.robot.util.scheduling.SubsystemPriority;
import org.jspecify.annotations.Nullable;

@SuppressWarnings("unused")
public class Swerve extends StateMachineSubsystem<SwerveState> implements PowerManaged {

  public static final double TRANSLATION_STD_DEV = 0.01;

  public static final double MAX_LINEAR_RATE = 4.75;
  private static final DoubleSubscriber MAX_LINEAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxLinearRateShooting", 4.0);

  private static final double MAX_ANGULAR_RATE = Units.rotationsToRadians(4);
  private static final DoubleSubscriber MAX_ANGULAR_RATE_SHOOTING =
      DogLog.tunable("Swerve/MaxAngularRateShootingRot", 0.8);
  public static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = Units.millisecondsToSeconds(5);

  private final CircularFilter lastDriveDirectionFilter = new CircularFilter(1);
  private final SlewRateLimiter maxLinearVelocityRateLimiter = new SlewRateLimiter(5.0);
  private final SlewRateLimiter maxAngularVelocityRateLimiter = new SlewRateLimiter(5.0);

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      new PhoenixPIDController(5.75, 0.0, 0);

  public final TunerSwerveDrivetrain drivetrain;

  private final XboxControllerDriveSource teleopDriveSource;
  private final TrailblazerDriveSource trailblazerDriveSource;
  private DriveSource driveSource;

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}. */
  private final SwerveRequest.FieldCentric driverPerspectiveOpenLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.05);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.0)
          .withRotationalDeadband(0.0)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#DRIVER_PERSPECTIVE_OPEN_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle drivePerspectiveIntakeSnapsOpenLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(MAX_ANGULAR_RATE);

  /** A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}. */
  private final SwerveRequest.FieldCentric fieldCentricClosedLoop =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  /**
   * A {@link SwerveRequest} for use with {@link DriveSourceType#FIELD_CENTRIC_CLOSED_LOOP}, but
   * overrides the angular velocity to instead snap to an angle.
   */
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricIntakeSnapsClosedLoop =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDeadband(0.07)
          .withRotationalDeadband(0.5)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(),
              ORIGINAL_HEADING_PID.getI(),
              ORIGINAL_HEADING_PID.getD());

  private final SwerveRequest xRequest = new SmoothX();

  private final HealthManager health;

  private double lastSimTime;
  private @Nullable Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private double turretStuckAimingAngle = 0.0;

  private boolean ableToBumpAssist = false;
  private double shootingSnapSetpoint = 0.0;
  private final double AIMED_TOLERANCE = 1.5;
  private final Debouncer X_SWERVE_DEBOUNCER = new Debouncer(0.25);
  private boolean ableToXSwerve = false;

  private double aimingFeedForward = 0.0;

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
    this.trailblazerDriveSource =
        new TrailblazerDriveSource(
            trailblazer, () -> drivetrainState.Pose, this::getFieldRelativeSpeeds);
    driveSource = teleopDriveSource;

    drivePerspectiveSnapsOpenLoop.HeadingController.setTolerance(0.0, 0.0);
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

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public void climbAssistDriveRequest() {
    setStateFromRequest(SwerveState.CLIMB_ASSIST);
  }

  public void rateLimitedDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL_RATE_LIMITED);
  }

  public void intakeRateLimitedDriveRequest() {
    setStateFromRequest(SwerveState.INTAKE_RATE_LIMITED);
  }

  public void turretStuckAimRequest(double snapAngle, double feedForward) {
    turretStuckAimingAngle = snapAngle;
    aimingFeedForward = feedForward;
    setStateFromRequest(SwerveState.TURRET_STUCK_SCORE);
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, drivetrainState.Pose.getRotation());

    shootingSnapSetpoint =
        switch (getState()) {
          case TURRET_STUCK_SCORE ->
              Math.toDegrees(MathUtil.angleModulus(Math.toRadians(turretStuckAimingAngle)));
          default -> 0.0;
        };
    ableToXSwerve =
        switch (getState()) {
          case TURRET_STUCK_SCORE -> {
            yield X_SWERVE_DEBOUNCER.calculate(
                FeatureFlags.X_SWERVE.getAsBoolean()
                    && MathUtil.isNear(
                        shootingSnapSetpoint,
                        drivetrainState.Pose.getRotation().getDegrees(),
                        AIMED_TOLERANCE,
                        -180.0,
                        180.0)
                    && MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()) < 1e-5);
          }
          default -> false;
        };

    ableToBumpAssist =
        DSOptions.USE_BUMP_ASSIST.get()
            && driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
            && health.isLocalizationHealthy()
            && SwerveAssist.ableToBumpAssist(drivetrainState.Pose, fieldRelativeSpeeds);

    var currentDriveDirection =
        MathHelpers.getDriveDirection(driveSource.getRequestedSpeeds())
            .plus(Rotation2d.fromDegrees(FmsUtil.isRedAlliance() ? 180 : 0));

    var requestedSpeeds = driveSource.getRequestedSpeeds();
    if (getState() == SwerveState.INTAKE_RATE_LIMITED
        || getState() == SwerveState.MANUAL_RATE_LIMITED) {
      double maxAngularRateRotations =
          maxAngularVelocityRateLimiter.calculate(MAX_ANGULAR_RATE_SHOOTING.get());
      teleopDriveSource.setMaxVelocity(
          maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE_SHOOTING.get()),
          Rotation2d.fromRotations(maxAngularRateRotations));

      drivePerspectiveIntakeSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      drivePerspectiveSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));

    } else {
      double maxAngularRateRotations =
          maxAngularVelocityRateLimiter.calculate(TELEOP_MAX_ANGULAR_RATE.getRotations());
      teleopDriveSource.setMaxVelocity(
          maxLinearVelocityRateLimiter.calculate(MAX_LINEAR_RATE),
          Rotation2d.fromRotations(maxAngularRateRotations));
      drivePerspectiveIntakeSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      drivePerspectiveSnapsOpenLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
      fieldCentricIntakeSnapsClosedLoop.withMaxAbsRotationalRate(
          Units.rotationsToRadians(maxAngularRateRotations));
    }
  }

  public void normalDriveRequest() {
    setStateFromRequest(SwerveState.MANUAL);
  }

  public void intakeDriveRequest() {
    setStateFromRequest(SwerveState.INTAKE);
  }

  private SwerveRequest.FieldCentricFacingAngle withFieldRelativeTargetDirection(
      SwerveRequest.FieldCentricFacingAngle request, Rotation2d targetDirection) {
    if (request.ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
      var snapSetpoint =
          FmsUtil.isRedAlliance()
              ? targetDirection.plus(drivetrain.getOperatorForwardDirection())
              : targetDirection;

      return request.withTargetDirection(snapSetpoint);
    }

    return request.withTargetDirection(targetDirection);
  }

  public boolean isAimed() {
    return MathUtil.isNear(
        turretStuckAimingAngle,
        drivetrainState.Pose.getRotation().getDegrees(),
        AIMED_TOLERANCE,
        -180,
        180);
  }

  @Override
  public void whileInState(SwerveState currentState) {
    drivetrain.setOperatorPerspectiveForward(
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero);

    switch (currentState) {
      case MANUAL, MANUAL_RATE_LIMITED -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveOpenLoop
                  : fieldCentricClosedLoop;

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        }
      }
      case INTAKE, INTAKE_RATE_LIMITED -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToBumpAssist) {
          drivetrain.setControl(
              withFieldRelativeTargetDirection(
                  drivePerspectiveSnapsOpenLoop
                      .withVelocityX(speeds.vxMetersPerSecond)
                      .withVelocityY(speeds.vyMetersPerSecond),
                  SwerveAssist.getRoundedSnapAngle(
                      drivetrainState.Pose.getRotation(), SwerveAssist.BUMP_SNAP_ROUND_ANGLE)));
        } else {
          var swerveRequest =
              driveSource.getDriveSourceType() == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP
                  ? driverPerspectiveOpenLoop
                  : fieldCentricClosedLoop;

          drivetrain.setControl(
              swerveRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        }
      }
      case TURRET_STUCK_SCORE -> {
        var speeds = driveSource.getRequestedSpeeds();

        if (ableToXSwerve) {
          drivetrain.setControl(xRequest);
          DogLog.timestamp("Swerve/XSwerveActive");
        } else if (driveSource.getDriveSourceType()
            == DriveSourceType.DRIVER_PERSPECTIVE_OPEN_LOOP) {
          DogLog.timestamp("Swerve/DriverOverridingRotation");
          if (driveSource.getRequestedSpeeds().omegaRadiansPerSecond > 1e-5) {
            drivetrain.setControl(
                driverPerspectiveOpenLoop
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond));
          } else {
            DogLog.timestamp("Swerve/TryingToAim");
            drivetrain.setControl(
                withFieldRelativeTargetDirection(
                        drivePerspectiveSnapsOpenLoop
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond),
                        Rotation2d.fromDegrees(turretStuckAimingAngle))
                    .withTargetRateFeedforward(aimingFeedForward));
          }
        }
      }
      case CLIMB_ASSIST -> {
        // Always use Trailblazer drive source for climb alignment
        var speeds = trailblazerDriveSource.getRequestedSpeeds();

        drivetrain.setControl(
            fieldCentricClosedLoop
                .withVelocityX(speeds.vxMetersPerSecond)
                .withVelocityY(speeds.vyMetersPerSecond)
                .withRotationalRate(speeds.omegaRadiansPerSecond));

        if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
          var current = drivetrainState.ModuleStates;
          var targets = drivetrainState.ModuleTargets;
          boolean isMisaligned = false;

          for (int i = 0; i < current.length; i++) {
            var actual = current[i];
            var target = targets[i];

            if (MathUtil.isNear(
                actual.angle.getDegrees(), target.angle.getDegrees(), 5, -180, 180)) {
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
        }
      }
    }

    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log(
        "Swerve/SwerveTargetDirection", drivePerspectiveSnapsOpenLoop.TargetDirection.getDegrees());
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);
    DogLog.log("Swerve/FieldRelativeSpeeds", fieldRelativeSpeeds);
    DogLog.log("Swerve/AbleToBumpAssist", ableToBumpAssist);
    DogLog.log("Swerve/GoalAimingAngle", turretStuckAimingAngle);
    // Temporary logging for X swerve feature flag
    if (FeatureFlags.X_SWERVE.getAsBoolean()) {
      DogLog.log("Swerve/X/AbleToXSwerve", ableToXSwerve);
      DogLog.log(
          "Swerve/X/SpeedsNear0",
          MathHelpers.getLinearVelocity(driveSource.getRequestedSpeeds()) < 1e-5);
      DogLog.log(
          "Swerve/X/ManualAtSetpoint",
          MathUtil.isNear(
              shootingSnapSetpoint,
              drivetrainState.Pose.getRotation().getDegrees(),
              AIMED_TOLERANCE,
              -180.0,
              180.0));
      DogLog.log("Swerve/X/ManualAtSetpoint/ControllerSetpoint", shootingSnapSetpoint);
      DogLog.log(
          "Swerve/X/ManualAtSetpoint/RobotHeading",
          drivetrainState.Pose.getRotation().getDegrees());
    }
    DogLog.log("Swerve/FeedForward", aimingFeedForward, Radians);
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
  public void applyCurrentLimits(double supplyCurrentLimit) {
    drivetrain
        .getModule(0)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.FrontLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(1)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.FrontRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
    drivetrain
        .getModule(2)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.BackLeft.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));

    drivetrain
        .getModule(3)
        .getDriveMotor()
        .getConfigurator()
        .apply(
            CompTunerConstants.BackRight.DriveMotorInitialConfigs.CurrentLimits
                .withSupplyCurrentLimit(supplyCurrentLimit));
  }
}
