package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.math.ShootOnTheMove.ShotSolution;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {

  public record AimingParameters(
      double turretAngle,
      double distance,
      double turretTolerance,
      double turretFeedForwardRadians,
      double upcomingTurretAngle,
      boolean solutionValid) {
    public AimingParameters(
        double turretAngle,
        double distance,
        double turretTolerance,
        double turretFeedForwardRadians,
        double upcomingTurretAngle) {
      this(
          turretAngle,
          distance,
          turretTolerance,
          turretFeedForwardRadians,
          upcomingTurretAngle,
          true);
    }
  }

  private record Model(double horizon, double linearTau, double angularTau, double drag) {
    static Model current() {
      return new Model(
          ShootOnTheMove.PREDICTION_HORIZON_SECONDS.getAsDouble(),
          VELOCITY_TIME_CONSTANT_SECONDS.getAsDouble(),
          ShootOnTheMove.ANGULAR_TIME_CONSTANT_SECONDS.getAsDouble(),
          ShootOnTheMove.DRAG_CONSTANT.getAsDouble());
    }

    ChassisSpeeds advanceSpeeds(ChassisSpeeds measured, ChassisSpeeds commanded, double dt) {
      return new ChassisSpeeds(
          ShootOnTheMove.predictVelocity(
              measured.vxMetersPerSecond, commanded.vxMetersPerSecond, dt, linearTau),
          ShootOnTheMove.predictVelocity(
              measured.vyMetersPerSecond, commanded.vyMetersPerSecond, dt, linearTau),
          ShootOnTheMove.predictVelocity(
              measured.omegaRadiansPerSecond, commanded.omegaRadiansPerSecond, dt, angularTau));
    }

    Pose2d advancePose(Pose2d pose, ChassisSpeeds measured, ChassisSpeeds commanded, double dt) {
      return new Pose2d(
          pose.getTranslation()
              .plus(
                  new Translation2d(
                      displacement(
                          measured.vxMetersPerSecond, commanded.vxMetersPerSecond, dt, linearTau),
                      displacement(
                          measured.vyMetersPerSecond, commanded.vyMetersPerSecond, dt, linearTau))),
          pose.getRotation()
              .plus(
                  Rotation2d.fromRadians(
                      displacement(
                          measured.omegaRadiansPerSecond,
                          commanded.omegaRadiansPerSecond,
                          dt,
                          angularTau))));
    }
  }

  // Provisional comp Q64 response estimate; see docs/swerve-lag-q64.md.
  // Keep this bot's calibration separate from the shared/competition default.
  public static final DoubleSubscriber VELOCITY_TIME_CONSTANT_SECONDS =
      DogLog.tunable("Offseason/ShootOnTheMove/VelocityTimeConstantSeconds", 0.30);

  public static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig::getScoreTimeOfFlight, "Scoring");

  public static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig::getFeedTimeOfFlight, "Feeding");

  private static final double FEEDING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(20);
  private static final double SCORING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(5);

  private static final double MIN_TURRET_TOLERANCE_DEGREES = 3.0;

  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  private static final double FEEDING_FALLBACK_TOLERANCE = 5.0;

  // This is a numerical derivative step, not a scheduler period or a latency compensation.
  private static final double DERIVATIVE_STEP_SECONDS = 0.0001;

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var fieldGoal = FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    var turretAngle = fieldGoal.minus(robotRotation).getDegrees();

    return new AimingParameters(
        turretAngle,
        FEEDING_FALLBACK_DISTANCE_TO_GOAL,
        FEEDING_FALLBACK_TOLERANCE,
        0.0,
        turretAngle);
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      ChassisSpeeds driverCommandedVelocity) {
    return getAimingParameters(
        FEEDING_SOTM,
        feedLocation.getTranslation(),
        FEEDING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, ChassisSpeeds driverCommandedVelocity) {
    return getAimingParameters(
        SCORING_SOTM,
        FieldUtil.HUB_POSE.getTranslation(),
        SCORING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity);
  }

  public static AimingParameters getStaticFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      ChassisSpeeds driverCommandedVelocity) {
    return getStaticAimingParameters(
        FEEDING_SOTM,
        feedLocation.getTranslation(),
        FEEDING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity);
  }

  public static AimingParameters getStaticScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, ChassisSpeeds driverCommandedVelocity) {
    return getStaticAimingParameters(
        SCORING_SOTM,
        FieldUtil.HUB_POSE.getTranslation(),
        SCORING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity);
  }

  /** Integral of the first-order velocity response to a held field-relative command. */
  private static double displacement(double measured, double commanded, double dt, double tau) {
    if (!Double.isFinite(commanded) || !Double.isFinite(tau) || tau <= 0.0) {
      return measured * dt;
    }
    return commanded * dt + (measured - commanded) * tau * -Math.expm1(-dt / tau);
  }

  private static AimingParameters getAimingParameters(
      ShootOnTheMove sotm,
      Translation2d goalTranslation,
      double tolerance,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      ChassisSpeeds driverCommandedVelocity) {
    return calculateParameters(
        sotm,
        goalTranslation,
        tolerance,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity,
        true);
  }

  private static AimingParameters getStaticAimingParameters(
      ShootOnTheMove sotm,
      Translation2d goalTranslation,
      double tolerance,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      ChassisSpeeds driverCommandedVelocity) {
    return calculateParameters(
        sotm,
        goalTranslation,
        tolerance,
        robotPose,
        fieldRelativeSpeeds,
        driverCommandedVelocity,
        false);
  }

  private static ShotSolution solutionAt(
      ShootOnTheMove sotm,
      Translation2d goal,
      Pose2d pose,
      ChassisSpeeds measured,
      ChassisSpeeds commanded,
      Model model,
      boolean moving) {
    // Predict at the robot center, then add omega cross r at the turret. This allows
    // different translational/angular response constants without mixing reference points.
    var predicted = model.advanceSpeeds(measured, commanded, model.horizon());
    var turretVelocity =
        TurretCalculator.getTurretChassisSpeeds(predicted, pose.getRotation().getDegrees());
    return sotm.solve(
        TurretCalculator.getTurretPose(pose).getTranslation(),
        goal,
        moving ? turretVelocity : new ChassisSpeeds(),
        model.drag());
  }

  static AimingParameters calculateParameters(
      ShootOnTheMove sotm,
      Translation2d goal,
      double tolerance,
      Pose2d pose,
      ChassisSpeeds measured,
      ChassisSpeeds commanded,
      boolean moving) {
    var model = Model.current();
    var solution = solutionAt(sotm, goal, pose, measured, commanded, model, moving);
    var origin = TurretCalculator.getTurretPose(pose).getTranslation();
    double distance = origin.getDistance(solution.goal());
    double angle = TurretCalculator.calculateTurretAimingAngle(pose, solution.goal());

    // Differentiate the actual compensated target, including changing TOF, robot
    // acceleration, rotating turret offset, and robot-relative heading subtraction.
    var nextPose = model.advancePose(pose, measured, commanded, DERIVATIVE_STEP_SECONDS);
    var nextSpeeds = model.advanceSpeeds(measured, commanded, DERIVATIVE_STEP_SECONDS);
    var nextSolution = solutionAt(sotm, goal, nextPose, nextSpeeds, commanded, model, moving);
    double nextAngle = TurretCalculator.calculateTurretAimingAngle(nextPose, nextSolution.goal());
    double feedforward =
        distance > 1e-6
            ? MathUtil.angleModulus(Math.toRadians(nextAngle - angle)) / DERIVATIVE_STEP_SECONDS
            : 0.0;

    var futurePose = model.advancePose(pose, measured, commanded, 0.5);
    var futureSpeeds = model.advanceSpeeds(measured, commanded, 0.5);
    var futureSolution = solutionAt(sotm, goal, futurePose, futureSpeeds, commanded, model, moving);
    double upcomingAngle =
        moving
            ? TurretCalculator.calculateTurretAimingAngle(futurePose, futureSolution.goal())
            : angle;
    boolean valid =
        Double.isFinite(model.horizon())
            && model.horizon() >= 0.0
            && Double.isFinite(model.linearTau())
            && model.linearTau() > 0.0
            && Double.isFinite(model.angularTau())
            && model.angularTau() > 0.0
            && solution.valid()
            && nextSolution.valid()
            && (!moving || futureSolution.valid())
            && Double.isFinite(feedforward);
    var parameters =
        new AimingParameters(
            angle,
            distance,
            Math.max(
                MIN_TURRET_TOLERANCE_DEGREES,
                TurretCalculator.getGoalCentricTurretTolerance(solution.goal(), pose, tolerance)),
            valid ? feedforward : 0.0,
            upcomingAngle,
            valid);

    String key = sotm.getLogPrefix();
    // A single timestamp identifies the inputs used in this solution, in both logging modes.
    DogLog.log(key + "/TimestampSeconds", Timer.getFPGATimestamp());
    DogLog.log(key + "/RobotPose", pose);
    DogLog.log(key + "/Goal", new Pose2d(goal, Rotation2d.kZero));
    DogLog.log(key + "/MeasuredFieldRelativeSpeeds", measured);
    DogLog.log(key + "/CommandedFieldRelativeSpeeds", commanded);
    DogLog.log(
        key + "/PredictedFieldRelativeSpeeds",
        model.advanceSpeeds(measured, commanded, model.horizon()));
    DogLog.log(key + "/PredictionHorizonSeconds", model.horizon());
    DogLog.log(key + "/LinearTimeConstantSeconds", model.linearTau());
    DogLog.log(key + "/AngularTimeConstantSeconds", model.angularTau());
    DogLog.log(key + "/DragPerSecond", model.drag());
    DogLog.log(key + "/MovingSolution", moving);
    DogLog.log(key + "/CompensatedGoal", new Pose2d(solution.goal(), Rotation2d.kZero));
    DogLog.log(key + "/TimeOfFlightSeconds", solution.timeOfFlight());
    DogLog.log(key + "/EffectiveTimeOfFlightSeconds", solution.effectiveTimeOfFlight());
    DogLog.log(key + "/ResidualMeters", solution.residualMeters());
    DogLog.log(key + "/SolutionValid", valid);
    DogLog.log(key + "/TofRegressionEnabled", FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean());
    DogLog.log(key + "/ShotTableRegressionEnabled", FeatureFlags.REGRESSION_MODEL.getAsBoolean());
    DogLog.log(key + "/Parameters", parameters);
    return parameters;
  }
}
