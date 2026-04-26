package frc.robot.cluster_map;

import static com.google.common.collect.ImmutableMap.toImmutableMap;
import static java.util.Map.Entry.comparingByValue;

import com.team581.math.GamePieceDetectionCalculator;
import com.team581.math.MathHelpers;
import com.team581.util.FieldUtil;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.limelight.LimelightHelpers;
import com.team581.vision.results.GamePieceResult;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.Localization;
import frc.robot.swerve.Swerve;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;

public class ClusterMap extends StateMachineSubsystem<ClusterMapState> {
  private static final double SAME_CLUSTER_DETECTION_THRESHOLD_METERS = 1.0;
  private static final double SWERVE_MAX_LINEAR_SPEED_TRACKING = 4.0;
  private static final double SWERVE_MAX_ANGULAR_SPEED_TRACKING = 150.0;
  private static final double CLUSTER_LIFETIME_SECONDS = 2.0;

  private static final double MIN_BALLS_PER_SECOND_THRESHOLD = 10;
  private static final double ESTIMATED_DRIVE_SPEED_MPS = 4.0;
  private static final double PICKUP_OVERHEAD_TIME_SEC = 0.5;

  // Limelight 3 V-FOV is 49.7 deg. Top edge is ~24.85 deg.
  // 10% of 49.7 is ~4.97. 24.85 - 4.97 = 19.88 deg.
  private static final double MAX_VALID_TY = 19.88;

  // Hard cap to prevent the inverse-square law from predicting thousands of balls
  private static final int MAX_CLUSTER_SIZE_CAP = 50;

  private static final double REFERENCE_BALL_AREA_AT_1M = calculateTheoreticalArea();

  private static final DoubleSubscriber SIMULATED_CLUSTER_X = DogLog.tunable("ClusterMapX", 9.0);
  private static final DoubleSubscriber SIMULATED_CLUSTER_Y = DogLog.tunable("ClusterMapY", 5.0);

  private static double calculateTheoreticalArea() {
    double fovX = Math.toRadians(63.3);
    double fovY = Math.toRadians(49.7);
    double resX = 640.0;
    double resY = 480.0;

    double ballDiameterMeters = 0.1524;

    double fx = resX / (2.0 * Math.tan(fovX / 2.0));
    double fy = resY / (2.0 * Math.tan(fovY / 2.0));

    double expectedWidthPx = fx * ballDiameterMeters;
    double expectedHeightPx = fy * ballDiameterMeters;

    return Math.PI * (expectedWidthPx / 2.0) * (expectedHeightPx / 2.0);
  }

  private Lane bestLane = Lane.LANE_0;
  private Optional<Pose2d> bestPose = Optional.empty();

  private boolean hasHighValueTrenchCluster = false;

  private final Limelight limelight;

  private final ArrayList<ClusterMapElement> clusterMap = new ArrayList<>();
  private double[] previousResult = new double[0];
  private boolean staleData = false;
  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds();
  private Localization localization;
  private Swerve swerve;

  private boolean deployFullyExtended = false;
  private boolean hasDoneWarmup = false;

  private final GamePieceResult gamePieceResult = new GamePieceResult();

  private final LaneSystem laneSystem =
      new LaneSystem(7.0, 9.86, 1.5, FieldUtil.FIELD_WIDTH_Y - 1.5, 2);

  private final OptionalVisionClusterData clusterDataResult = new OptionalVisionClusterData();

  private final Translation2d warmupTranslation = new Translation2d(1.0, 1.0);

  public ClusterMap(Localization localization, Swerve swerve, Limelight limelight) {
    super(SubsystemPriority.VISION, ClusterMapState.DEFAULT_STATE);
    this.localization = localization;
    this.swerve = swerve;
    this.limelight = limelight;
  }

  /** Returns the lane with the most detected balls, ignoring the trench. */
  public Lane getBestClusterLane() {
    return bestLane;
  }

  public Optional<Pose2d> getBestClusterPose() {
    return bestPose;
  }

  public boolean hasHighValueTrenchCluster() {
    return hasHighValueTrenchCluster;
  }

  public void setDeployFullyExtended(boolean isFullyExtended) {
    deployFullyExtended = isFullyExtended;
  }

  private Lane calculateBestClusterLane() {
    var robotPose = localization.getPose();
    DogLog.timestamp("ClusterMap/RanBestLane");

    return clusterMap.stream()
        // Sum up total detected balls per lane
        .collect(
            toImmutableMap(
                element ->
                    laneSystem.getLane(
                        new Pose2d(element.clusterTranslation(), Rotation2d.kZero), robotPose),
                element -> (int) element.detectionSize(),
                Integer::sum))
        .entrySet()
        .stream()
        // Only consider actual scoring lanes
        .filter(entry -> entry.getKey() != Lane.NONE && entry.getKey() != Lane.TRENCH)
        // Pick the lane with the highest ball count
        .max(comparingByValue())
        .map(Map.Entry::getKey)
        .orElse(Lane.NONE);
  }

  private Optional<Pose2d> calculateBestClusterPose() {

    if (clusterMap.isEmpty()) {
      DogLog.log("ClusterMap/BestClusterPose", Pose2d.kZero);
      return Optional.empty();
    }
    DogLog.timestamp("ClusterMap/RanBestPose");

    ClusterMapElement bestElement = null;
    double highestImmediateScore = 0.0;
    var robotPose = localization.getPose();

    for (ClusterMapElement element : clusterMap) {
      var translation = element.clusterTranslation();
      var rotationToTarget =
          MathHelpers.getDriveDirection(robotPose, new Pose2d(translation, Rotation2d.kZero));

      if (!MathUtil.isNear(
          robotPose.getRotation().getDegrees(), rotationToTarget.getDegrees(), 45, -180, 180)) {
        continue;
      }

      double distanceMeters = robotPose.getTranslation().getDistance(translation);

      double estimatedTravelTime = distanceMeters / ESTIMATED_DRIVE_SPEED_MPS;
      double totalEstimatedTime = estimatedTravelTime + PICKUP_OVERHEAD_TIME_SEC;
      double ballsPerSecond = element.detectionSize() / totalEstimatedTime;

      double immediateScore = ballsPerSecond / Math.max(0.5, distanceMeters);

      if (ballsPerSecond >= MIN_BALLS_PER_SECOND_THRESHOLD
          && immediateScore > highestImmediateScore) {
        highestImmediateScore = immediateScore;
        bestElement = element;
      }
    }

    if (bestElement == null) {
      DogLog.timestamp("ClusterMap/No valid front-facing target met threshold");
      DogLog.log("ClusterMap/BestClusterPose", Pose2d.kZero);
      return Optional.empty();
    }

    var bestTranslation = bestElement.clusterTranslation();
    var finalRotation =
        MathHelpers.getDriveDirection(robotPose, new Pose2d(bestTranslation, Rotation2d.kZero));

    var clusterPoseWithIntakeRotation = new Pose2d(bestTranslation, finalRotation);
    DogLog.log("ClusterMap/BestClusterPose", clusterPoseWithIntakeRotation);

    return Optional.of(clusterPoseWithIntakeRotation);
  }

  private boolean calculateHasHighValueCluster() {
    if (clusterMap.isEmpty()) {
      return false;
    }
    DogLog.timestamp("ClusterMap/RanHighValueCluster");

    var robotPose = localization.getPose();

    for (ClusterMapElement element : clusterMap) {
      Pose2d elementPose = new Pose2d(element.clusterTranslation(), Rotation2d.kZero);

      // Only evaluate clusters in the trench
      if (laneSystem.getLane(elementPose, robotPose) == Lane.TRENCH) {

        double distanceMeters =
            robotPose.getTranslation().getDistance(element.clusterTranslation());
        double estimatedTravelTime = distanceMeters / ESTIMATED_DRIVE_SPEED_MPS;
        double totalEstimatedTime = estimatedTravelTime + PICKUP_OVERHEAD_TIME_SEC;

        double ballsPerSecond = element.detectionSize() / totalEstimatedTime;

        // Make sure we're confident in choosing this cluster
        if (ballsPerSecond > 27.0) {
          return true;
        }
      }
    }

    return false;
  }

  private OptionalVisionClusterData getRawClusterPoses() {
    if (RobotBase.isSimulation()) {
      return clusterDataResult.update(
          new Translation2d(SIMULATED_CLUSTER_X.getAsDouble(), SIMULATED_CLUSTER_Y.getAsDouble()),
          40,
          10);
    }
    if (!hasDoneWarmup) {
      return clusterDataResult.update(warmupTranslation, 20, 20);
    }
    if (limelight.getState() != LimelightState.CLUSTER_MAP && !deployFullyExtended) {
      return clusterDataResult.empty();
    }

    double[] result = LimelightHelpers.getPythonScriptData(limelight.limelightTableName);

    if (result == null || result.length < 5) {
      DogLog.timestamp("ClusterMap/NoData");
      return clusterDataResult.empty();
    }

    staleData = Arrays.equals(previousResult, result);
    previousResult = result;
    if (staleData) {
      DogLog.timestamp("ClusterMap/SkipStaleData");
      return clusterDataResult.empty();
    }

    double angleY = LimelightHelpers.getTY(limelight.limelightTableName);

    if (angleY > MAX_VALID_TY) {
      DogLog.timestamp("ClusterMap/RejectedHighTY");
      return clusterDataResult.empty();
    }

    double rawArea = result[2];
    double clusterScore = result[4];

    double latency =
        (LimelightHelpers.getLatency_Capture(limelight.limelightTableName)
                + LimelightHelpers.getLatency_Pipeline(limelight.limelightTableName))
            / 1000.0;

    double timestamp = Timer.getFPGATimestamp() - latency;
    var robotPoseAtCapture = localization.getPose(timestamp);
    double angleX = LimelightHelpers.getTX(limelight.limelightTableName);

    gamePieceResult.update(angleX, angleY, timestamp);

    // Calculate the absolute field position of the cluster
    var clusterPose =
        GamePieceDetectionCalculator.calculateFieldRelativeTranslationFromCamera(
            robotPoseAtCapture, gamePieceResult, limelight.config);

    if (Double.isNaN(clusterPose.getX()) || Double.isNaN(clusterPose.getY())) {
      return clusterDataResult.empty();
    }

    double distanceMeters = robotPoseAtCapture.getTranslation().getDistance(clusterPose);

    // Dynamic Size Estimation
    double estimatedBalls = (rawArea * Math.pow(distanceMeters, 2)) / REFERENCE_BALL_AREA_AT_1M;

    int calculatedSize = (int) Math.round(estimatedBalls);
    calculatedSize = MathUtil.clamp(calculatedSize, 1, MAX_CLUSTER_SIZE_CAP);

    return clusterDataResult.update(clusterPose, calculatedSize, clusterScore);
  }

  private boolean safeToTrack() {
    return swerveSpeeds.vxMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.vyMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.omegaRadiansPerSecond < Math.toRadians(SWERVE_MAX_ANGULAR_SPEED_TRACKING);
  }

  private void updateMap() {
    var latestData = getRawClusterPoses();
    if (latestData.isEmpty() || !safeToTrack()) {
      return;
    }

    var visionData = latestData.orElseThrow();
    var visionTranslation = visionData.translation();
    var visionSize = visionData.size();
    var visionScore = visionData.score();

    clusterMap.removeIf(element -> element.expiresAt() < Timer.getFPGATimestamp());

    if (staleData) {
      return;
    }

    double newClusterExpiry = Timer.getFPGATimestamp() + CLUSTER_LIFETIME_SECONDS;

    Optional<ClusterMapElement> match =
        clusterMap.stream()
            .filter(
                rememberedCluster -> {
                  return rememberedCluster.expiresAt() != newClusterExpiry
                      && (rememberedCluster.clusterTranslation().getDistance(visionTranslation)
                          < SAME_CLUSTER_DETECTION_THRESHOLD_METERS);
                })
            .min(
                (a, b) ->
                    Double.compare(
                        a.clusterTranslation().getDistance(visionTranslation),
                        b.clusterTranslation().getDistance(visionTranslation)));

    if (match.isPresent()) {
      var existingElement = match.orElseThrow();
      var health = Math.min(existingElement.health() + 1, 20);

      // Blend the old position with the newly observed position
      var blendedPose =
          existingElement.clusterTranslation().interpolate(visionTranslation, 1 / health);

      clusterMap.add(
          new ClusterMapElement(
              newClusterExpiry,
              blendedPose,
              health,
              Math.max(existingElement.detectionSize(), visionSize),
              visionScore));
      clusterMap.remove(existingElement);
    } else {
      clusterMap.add(
          new ClusterMapElement(newClusterExpiry, visionTranslation, 1.0, visionSize, visionScore));
    }
  }

  @Override
  protected void collectInputs() {
    if (FeatureFlags.CLUSTER_MAP.getAsBoolean() && DriverStation.isAutonomous()) {
      swerveSpeeds = swerve.getRobotRelativeSpeeds();
      updateMap();
      bestLane = calculateBestClusterLane();
      bestPose = calculateBestClusterPose();
      hasHighValueTrenchCluster = calculateHasHighValueCluster();
      hasDoneWarmup = true;
    }
  }

  @Override
  protected void whileInState(ClusterMapState state) {
    if (DriverStation.isAutonomous() && FeatureFlags.CLUSTER_MAP.getAsBoolean()) {

      try {
        DogLog.log("ClusterMap/Clusters", clusterMap.stream().toArray(ClusterMapElement[]::new));
        DogLog.log(
            "ClusterMap/Clusters/ClusterPoses",
            clusterMap.stream()
                .map(l -> new Pose2d(l.clusterTranslation(), Rotation2d.kZero))
                .toArray(Pose2d[]::new));
      } catch (RuntimeException error) {
        DogLog.logFault("ClusterMapLoggingError");
        System.err.println(error);
      }
    }
    DogLog.log("ClusterMap/HasDoneWarmup", hasDoneWarmup);
  }
}
