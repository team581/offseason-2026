package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.google.common.collect.ImmutableList;
import com.team581.spot.Spot;
import com.team581.spot.SpotCameraStatus;
import com.team581.spot.SpotEstimate;
import com.team581.spot.SpotStatus;
import com.team581.util.AprilTags;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.health.HealthManager;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class VisionTest {
  /** Plays Spot's side of NetworkTables on a private instance. */
  static final class Fixture implements AutoCloseable {
    double time = 10.;
    boolean operator = true;
    Vision.Motion motion = new Vision.Motion(0, 0, 0);
    final Pose2d robot = new Pose2d(4, 4, Rotation2d.kZero);
    Pose2d odometry = robot;
    final NetworkTableInstance nt = NetworkTableInstance.create();
    final StructPublisher<SpotStatus> status =
        nt.getStructTopic("/Spot/outputs/status", SpotStatus.struct).publish();
    final Map<String, StructPublisher<SpotEstimate>> estimates = new HashMap<>();
    final Map<String, StructPublisher<SpotCameraStatus>> cameraStatus = new HashMap<>();
    final Vision vision;

    Fixture() {
      for (var name : CAMERAS) {
        var prefix = "/Spot/outputs/cameras/" + name + "/";
        estimates.put(
            name,
            nt.getStructTopic(prefix + "estimate", SpotEstimate.struct)
                .publish(PubSubOption.keepDuplicates(true)));
        cameraStatus.put(
            name, nt.getStructTopic(prefix + "status", SpotCameraStatus.struct).publish());
        cameraStatus.get(name).set(new SpotCameraStatus(SpotCameraStatus.State.STREAMING, 120, 0));
      }
      status.set(new SpotStatus("test", SpotStatus.State.READY, false));
      vision =
          new Vision(
              new Spot(nt, "", AprilTags.FIELD_LAYOUT),
              CAMERAS,
              now -> Optional.of(odometry),
              () -> motion,
              () -> time,
              () -> operator);
      vision.collectInputs();
    }

    @Override
    public void close() {
      vision.close();
      status.close();
      estimates.values().forEach(StructPublisher::close);
      cameraStatus.values().forEach(StructPublisher::close);
      nt.close();
    }

    /** Publishes a two-tag estimate of {@link #robot}. */
    void estimate(String camera, double capture) {
      publish(
          camera,
          new SpotEstimate(
              new SpotEstimate.Candidate(new Pose3d(robot), .1),
              Optional.empty(),
              new int[] {1, 2},
              2,
              .02),
          capture);
    }

    void publish(String camera, SpotEstimate estimate, double capture) {
      estimates.get(camera).set(estimate, (long) (capture * 1e6));
      nt.waitForListenerQueue(1);
    }

    Vision.Reason reason(String camera) {
      return vision.reason(camera).orElseThrow();
    }

    void tick(double now) {
      time = now;
      vision.collectInputs();
      vision.robotPeriodic();
    }
  }

  private static final List<String> CAMERAS = ImmutableList.of("left", "right", "back");

  @BeforeAll
  static void hal() {
    HAL.initialize(500, 0);
  }

  @Test
  void camerasProduceMeasurementsOldestFirstAndDrainOnce() {
    try (var f = new Fixture()) {
      f.estimate("right", 10.03);
      f.estimate("left", 10.01);
      f.estimate("back", 10.02);
      f.tick(10.04);
      var batch = f.vision.drainMeasurements();
      assertThat(batch)
          .extracting(VisionMeasurement::timestamp)
          .containsExactly(10.01, 10.02, 10.03);
      for (var value : batch) {
        assertThat(value.pose().getX()).isCloseTo(4., offset(1e-9));
        assertThat(value.pose().getY()).isCloseTo(4., offset(1e-9));
      }
      assertThat(f.vision.correlationFactor()).isEqualTo(Math.sqrt(3));
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.tick(10.06);
      assertThat(f.vision.drainMeasurements()).isEmpty();
    }
  }

  @Test
  void headingAtCapturePicksBetweenSpotsCandidates() {
    try (var f = new Fixture()) {
      var flipped = new Pose3d(new Pose2d(4, 4, Rotation2d.k180deg));
      var right = new Pose3d(new Pose2d(5, 4, Rotation2d.kZero));
      f.publish(
          "left",
          new SpotEstimate(
              new SpotEstimate.Candidate(flipped, .1),
              Optional.of(new SpotEstimate.Candidate(right, .5)),
              new int[] {1},
              2,
              .02),
          10.01);
      f.tick(10.04);
      assertThat(f.vision.drainMeasurements())
          .singleElement()
          .satisfies(value -> assertThat(value.pose().getX()).isCloseTo(5., offset(1e-9)));

      f.publish(
          "left",
          new SpotEstimate(
              new SpotEstimate.Candidate(flipped, .1), Optional.empty(), new int[] {1}, 2, .02),
          10.05);
      f.tick(10.08);
      assertThat(f.reason("left")).isEqualTo(Vision.Reason.AMBIGUOUS);
      assertThat(f.vision.drainMeasurements()).isEmpty();
    }
  }

  @Test
  void innovationIsDistanceFromOdometryAtCapture() {
    try (var f = new Fixture()) {
      f.odometry = new Pose2d(0, 4, Rotation2d.kZero);
      f.estimate("left", 10.01);
      f.tick(10.04);
      assertThat(f.vision.drainMeasurements())
          .singleElement()
          .satisfies(value -> assertThat(value.innovationM()).isCloseTo(4., offset(1e-9)));
    }
  }

  @Test
  void localizationHealthHoldsThroughBriefTrustLossAndHonorsOperatorOverrides() {
    try (var f = new Fixture()) {
      var health = new HealthManager(f.vision, false);
      var trusted = new AtomicBoolean(false);
      health.setPoseTrustworthySupplier(trusted::get);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
      trusted.set(true);
      f.tick(10.04);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();
      assertThat(health.isAllCamerasHealthy()).isTrue();
      trusted.set(false);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();
      try (var use =
              NetworkTableInstance.getDefault().getBooleanTopic("/DSOptions/UseVision").publish();
          var pit =
              NetworkTableInstance.getDefault()
                  .getBooleanTopic("/DSOptions/PitFunctionality")
                  .publish()) {
        use.set(false);
        assertThat(health.isLocalizationHealthy()).isFalse();
        use.set(true);
        pit.set(true);
        assertThat(health.isLocalizationHealthy()).isFalse();
        pit.set(false);
        assertThat(health.isLocalizationHealthy()).isTrue();
      }
      f.tick(11.1);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
    }
  }

  @Test
  void nothingReachesTheEstimatorWhileFusionIsOffOrSpotIsDisconnected() {
    try (var f = new Fixture()) {
      f.operator = false;
      f.estimate("left", 10.01);
      f.tick(10.02);
      assertThat(f.vision.drainMeasurements()).isEmpty();

      f.operator = true;
      f.status.close();
      f.estimate("left", 10.03);
      f.tick(10.04);
      assertThat(f.vision.isSpotReady()).isFalse();
      assertThat(f.reason("left")).isEqualTo(Vision.Reason.SPOT_NOT_READY);
      assertThat(f.vision.drainMeasurements()).isEmpty();
    }
  }

  @Test
  void resetDiscardsEstimatesCapturedBeforeIt() {
    try (var f = new Fixture()) {
      f.estimate("left", 10.13);
      f.tick(10.16);
      assertThat(f.vision.drainMeasurements()).hasSize(1);
      f.vision.reset(10.16);
      f.estimate("left", 10.15);
      f.tick(10.18);
      assertThat(f.reason("left")).isEqualTo(Vision.Reason.RESET);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.estimate("left", 10.17);
      f.tick(10.19);
      assertThat(f.vision.drainMeasurements()).hasSize(1);
    }
  }

  @Test
  void spotNotReadyMeansCamerasAreUnhealthy() {
    try (var f = new Fixture()) {
      f.status.set(new SpotStatus("test", SpotStatus.State.MISSING_CAMERA, false));
      f.tick(10.04);
      assertThat(f.vision.isSpotReady()).isFalse();
    }
  }

  @Test
  void tiltedRobotRejectsEstimates() {
    try (var f = new Fixture()) {
      f.motion = new Vision.Motion(0, 0, Math.toRadians(20));
      f.estimate("left", 10.01);
      f.tick(10.04);
      assertThat(f.reason("left")).isEqualTo(Vision.Reason.ROBOT_TILT);
      assertThat(f.vision.drainMeasurements()).isEmpty();
    }
  }
}
