package frc.robot.cluster_map;

import com.team581.math.GamePieceDetectionCalculator;
import com.team581.math.MathHelpers;
import com.team581.util.FieldUtil;
import com.team581.util.profiling.DiagnosticCadence;
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
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import java.util.List;
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

  private static final int WARMUP_TARGET_TICKS = 1000;

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

  private static Lane scoringLane(int index) {
    return switch (index) {
      case 0 -> Lane.LANE_0;
      case 1 -> Lane.LANE_1;
      case 2 -> Lane.LANE_2;
      case 3 -> Lane.LANE_3;
      case 4 -> Lane.LANE_4;
      default -> Lane.NONE;
    };
  }

  private static int scoringLaneIndex(Lane lane) {
    return switch (lane) {
      case LANE_0 -> 0;
      case LANE_1 -> 1;
      case LANE_2 -> 2;
      case LANE_3 -> 3;
      case LANE_4 -> 4;
      case TRENCH, NONE -> -1;
    };
  }

  static Lane chooseBestLane(int[] counts, int[] firstSeen) {
    Lane best = Lane.NONE;
    int bestCount = Integer.MIN_VALUE;
    int bestFirstSeen = Integer.MAX_VALUE;
    for (int i = 0; i < counts.length; i++) {
      if (firstSeen[i] != Integer.MAX_VALUE
          && (counts[i] > bestCount || (counts[i] == bestCount && firstSeen[i] < bestFirstSeen))) {
        best = scoringLane(i);
        bestCount = counts[i];
        bestFirstSeen = firstSeen[i];
      }
    }
    return best;
  }

  static ClusterMapElement findClosestCluster(
      List<ClusterMapElement> clusters, Translation2d visionTranslation, double newClusterExpiry) {
    ClusterMapElement existingElement = null;
    double closestDistance = Double.POSITIVE_INFINITY;
    for (ClusterMapElement rememberedCluster : clusters) {
      double distance = rememberedCluster.clusterTranslation().getDistance(visionTranslation);
      if (rememberedCluster.expiresAt() != newClusterExpiry
          && distance < SAME_CLUSTER_DETECTION_THRESHOLD_METERS
          && distance < closestDistance) {
        existingElement = rememberedCluster;
        closestDistance = distance;
      }
    }
    return existingElement;
  }

  private Lane bestLane = Lane.LANE_0;
  private Optional<Pose2d> bestPose = Optional.empty();
  private final Limelight limelight;
  private final ArrayList<ClusterMapElement> clusterMap = new ArrayList<>();
  private double[] previousResult = new double[0];
  private boolean staleData = false;

  private ChassisSpeeds swerveSpeeds = new ChassisSpeeds();
  private Localization localization;
  private Swerve swerve;

  private boolean deployFullyExtended = false;

  private boolean hasDoneWarmup = false;

  private int warmupTickCount = 0;

  private final GamePieceResult gamePieceResult = new GamePieceResult();

  private final LaneSystem laneSystem =
      new LaneSystem(7.0, 9.86, 1.5, FieldUtil.FIELD_WIDTH_Y - 1.5, 2);

  private final OptionalVisionClusterData clusterDataResult = new OptionalVisionClusterData();

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

  public void setDeployFullyExtended(boolean isFullyExtended) {
    deployFullyExtended = isFullyExtended;
  }

  private Lane calculateBestClusterLane() {
    var robotPose = localization.getPose();
    DogLog.timestamp("ClusterMap/RanBestLane");

    int[] counts = new int[5];
    int[] firstSeen = {
      Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE
    };
    int encounterIndex = 0;
    for (ClusterMapElement element : clusterMap) {
      Lane lane =
          laneSystem.getLane(new Pose2d(element.clusterTranslation(), Rotation2d.kZero), robotPose);
      int laneIndex = scoringLaneIndex(lane);
      if (laneIndex >= 0) {
        counts[laneIndex] += (int) element.detectionSize();
        firstSeen[laneIndex] = Math.min(firstSeen[laneIndex], encounterIndex);
      }
      encounterIndex++;
    }
    return chooseBestLane(counts, firstSeen);
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

  private OptionalVisionClusterData getRawClusterPoses() {
    if (RobotBase.isSimulation()) {
      return clusterDataResult.update(
          new Translation2d(SIMULATED_CLUSTER_X.getAsDouble(), SIMULATED_CLUSTER_Y.getAsDouble()),
          40,
          10);
    }
    if (!hasDoneWarmup) {
      return getWarmupRawClusterPose();
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

  /**
   * Drives synthetic but realistic data through the same code path the real Limelight read uses, so
   * the JIT compiles every method we'll need before auto starts. Inputs are varied across ticks so
   * different downstream branches get exercised.
   */
  private OptionalVisionClusterData getWarmupRawClusterPose() {
    // Touch the real Limelight read methods so they get JITed too. Result may be null/empty
    // before the LL is publishing - we don't care, we just need the methods invoked. Exercise
    // the staleData comparison too (without actually using its result, since we want updateMap
    // to keep running so its blend/match branches get warmed up).
    double[] result = LimelightHelpers.getPythonScriptData(limelight.limelightTableName);
    if (result != null && previousResult != null) {
      var unused = Arrays.equals(previousResult, result);
      previousResult = result;
    }
    staleData = false;
    LimelightHelpers.getLatency_Capture(limelight.limelightTableName);
    LimelightHelpers.getLatency_Pipeline(limelight.limelightTableName);
    LimelightHelpers.getTX(limelight.limelightTableName);
    LimelightHelpers.getTY(limelight.limelightTableName);

    // Synthetic ty/tx/area that vary each tick to exercise different downstream branches.
    // Keep ty under MAX_VALID_TY so we don't always trip the rejection branch.
    double tickPhase = (warmupTickCount % 60) / 60.0;
    double syntheticTy = 5.0 + tickPhase * 10.0;
    double syntheticTx = -15.0 + tickPhase * 30.0;
    double syntheticArea = 50.0 + tickPhase * 200.0;

    double timestamp = Timer.getFPGATimestamp();
    var robotPoseAtCapture = localization.getPose(timestamp);

    gamePieceResult.update(syntheticTx, syntheticTy, timestamp);

    var clusterPose =
        GamePieceDetectionCalculator.calculateFieldRelativeTranslationFromCamera(
            robotPoseAtCapture, gamePieceResult, limelight.config);

    if (Double.isNaN(clusterPose.getX()) || Double.isNaN(clusterPose.getY())) {
      return clusterDataResult.empty();
    }

    double distanceMeters = robotPoseAtCapture.getTranslation().getDistance(clusterPose);
    double estimatedBalls =
        (syntheticArea * Math.pow(distanceMeters, 2)) / REFERENCE_BALL_AREA_AT_1M;
    int calculatedSize = (int) Math.round(estimatedBalls);
    calculatedSize = MathUtil.clamp(calculatedSize, 1, MAX_CLUSTER_SIZE_CAP);

    return clusterDataResult.update(clusterPose, calculatedSize, 10.0);
  }

  private boolean safeToTrack() {
    return swerveSpeeds.vxMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.vyMetersPerSecond < SWERVE_MAX_LINEAR_SPEED_TRACKING
        && swerveSpeeds.omegaRadiansPerSecond < Math.toRadians(SWERVE_MAX_ANGULAR_SPEED_TRACKING);
  }

  /**
   * Seed the cluster map with synthetic entries so {@link #updateMap()} hits the "match existing
   * cluster -> blend / interpolate" branch instead of only the "no match -> add new" branch, and so
   * the lane / trench / front-facing scoring code paths all see real data.
   */
  private void seedWarmupClusters() {
    if (!clusterMap.isEmpty()) {
      return;
    }
    double expiry = Timer.getFPGATimestamp() + CLUSTER_LIFETIME_SECONDS;
    Translation2d[] seeds = {
      new Translation2d(8.0, 1.0),
      new Translation2d(8.0, FieldUtil.FIELD_WIDTH_Y - 1.0),
      new Translation2d(8.5, FieldUtil.FIELD_WIDTH_Y / 2.0 - 1.0),
      new Translation2d(8.5, FieldUtil.FIELD_WIDTH_Y / 2.0 + 1.0),
      new Translation2d(9.5, FieldUtil.FIELD_WIDTH_Y / 2.0),
    };
    for (Translation2d seed : seeds) {
      clusterMap.add(new ClusterMapElement(expiry, seed, 5.0, 15.0, 5.0));
    }
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

    double now = Timer.getFPGATimestamp();
    clusterMap.removeIf(element -> element.expiresAt() < now);

    if (staleData) {
      return;
    }

    double newClusterExpiry = now + CLUSTER_LIFETIME_SECONDS;

    ClusterMapElement existingElement =
        findClosestCluster(clusterMap, visionTranslation, newClusterExpiry);

    if (existingElement != null) {
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

      // While disabled in the autonomous period, run warmup continuously so the JIT can
      // promote every method on the real cluster-map code path before the match starts.
      // Latching after a single tick (the previous behavior) only let HotSpot see each method
      // once, which left the actual auto code path being interpreted/compiled live - producing
      // the lag spike we see at the midline.
      if (DriverStation.isDisabled() && warmupTickCount < WARMUP_TARGET_TICKS) {
        seedWarmupClusters();
        updateMap();
        bestLane = calculateBestClusterLane();
        bestPose = calculateBestClusterPose();
        warmupTickCount++;
      } else {
        // First enabled tick: drop any synthetic data we seeded during warmup so we start
        // from a clean slate on real Limelight input.
        if (!hasDoneWarmup) {
          clusterMap.clear();
          previousResult = new double[0];
        }
        hasDoneWarmup = true;
        updateMap();
        bestLane = calculateBestClusterLane();
        bestPose = calculateBestClusterPose();
      }
    }
  }

  @Override
  protected void whileInState(ClusterMapState state) {
    if (DriverStation.isAutonomous() && FeatureFlags.CLUSTER_MAP.getAsBoolean()) {
      if (DiagnosticCadence.shouldLogHeavy()) {
        try {
          DogLog.log("ClusterMap/Clusters", clusterMap.toArray(ClusterMapElement[]::new));
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
    }
    if (DiagnosticCadence.shouldLogRoutine()) {
      DogLog.log("ClusterMap/HasDoneWarmup", hasDoneWarmup);
      DogLog.log("ClusterMap/WarmupTickCount", warmupTickCount);
    }

    if (warmupTickCount < WARMUP_TARGET_TICKS) {
      DogLog.logFault("Cluster map warmup still running", AlertType.kWarning);
    } else {
      DogLog.clearFault("Cluster map warmup still running");
    }
  }
}
