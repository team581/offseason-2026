package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.team581.spot.Spot;
import com.team581.spot.SpotCameraStatus;
import com.team581.spot.SpotEstimate;
import com.team581.spot.SpotStatus;
import com.team581.util.AprilTags;
import com.team581.vision.spot.VisionProcessor;
import com.team581.vision.spot.results.VisionMeasurement;
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
    final Pose2d robot = new Pose2d(4, 4, Rotation2d.kZero);
    Pose2d estimatedPose = robot;
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
        streaming(name, true);
      }
      status.set(new SpotStatus("test", SpotStatus.State.READY, false));
      vision =
          new Vision(
              new Spot(nt, "", AprilTags.FIELD_LAYOUT),
              CAMERAS,
              new Vision.CaptureSource() {
                @Override
                public VisionProcessor.CaptureState current(double now) {
                  return new VisionProcessor.CaptureState(estimatedPose, 0, 0, 0, 0, 0);
                }

                @Override
                public Optional<Pose2d> poseAt(double now) {
                  return Optional.of(estimatedPose);
                }
              },
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

    /** Publishes an estimate of {@link #robot} from tag 1 and {@code tag}. */
    void estimate(String camera, int tag, double capture) {
      var ids = new int[] {1, tag};
      estimates
          .get(camera)
          .set(
              new SpotEstimate(
                  new SpotEstimate.Candidate(new Pose3d(robot), .1), Optional.empty(), ids, 2, .02),
              (long) (capture * 1e6));
      nt.waitForListenerQueue(1);
    }

    VisionProcessor.Reason reason(String camera) {
      return VisionProcessor.Reason.valueOf(vision.getStatus().cameras().get(camera).reason());
    }

    void streaming(String camera, boolean streaming) {
      cameraStatus
          .get(camera)
          .set(
              new SpotCameraStatus(
                  streaming
                      ? SpotCameraStatus.State.STREAMING
                      : SpotCameraStatus.State.DISCONNECTED,
                  120,
                  0));
    }

    void tick(double now) {
      time = now;
      vision.collectInputs();
      vision.robotPeriodic();
    }
  }

  private static final List<String> CAMERAS = List.of("left", "right", "back");

  @BeforeAll
  static void hal() {
    HAL.initialize(500, 0);
  }

  @Test
  void camerasRecoverSamePoseAndDrainOnce() {
    try (var f = new Fixture()) {
      f.estimate("left", 2, 10.01);
      f.estimate("right", 3, 10.01);
      f.estimate("back", 5, 10.01);
      f.tick(10.04);
      var batch = f.vision.drainMeasurements();
      assertThat(batch)
          .extracting(VisionMeasurement::source)
          .containsExactlyInAnyOrder("left", "right", "back");
      assertThat(f.vision.correlationFactor()).isEqualTo(Math.sqrt(3));
      assertThat(f.vision.drainMeasurements()).isEmpty();
      for (var value : batch) {
        assertThat(value.pose().getX()).isCloseTo(4., offset(1e-9));
        assertThat(value.pose().getY()).isCloseTo(4., offset(1e-9));
        assertThat(value.timestamp()).isCloseTo(10.01, offset(1e-6));
        assertThat(value.stdHeading()).isEqualTo(1e6);
        assertThat(f.vision.eligible(value, 10.04)).isTrue();
        f.vision.measurementFused(value);
      }
      assertThat(f.vision.getStatus().allRequiredHealthy()).isTrue();
      assertThat(f.vision.hasRecentFusedMeasurement()).isTrue();
      f.tick(10.06);
      assertThat(f.vision.drainMeasurements()).isEmpty();
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
  void shadowFusionAndResetCannotClaimLocalizationReadiness() {
    try (var shadow = new Fixture()) {
      shadow.operator = false;
      shadow.estimate("left", 2, 10.01);
      shadow.tick(10.04);
      assertThat(shadow.vision.isFusionEnabled()).isFalse();
      assertThat(shadow.vision.hasRecentFusedMeasurement()).isFalse();
    }
    try (var active = new Fixture()) {
      active.estimate("left", 2, 10.01);
      active.tick(10.04);
      active.vision.drainMeasurements().forEach(active.vision::measurementFused);
      assertThat(active.vision.hasRecentFusedMeasurement()).isTrue();
      active.vision.reset(10.04);
      assertThat(active.vision.hasRecentFusedMeasurement()).isFalse();
    }
  }

  @Test
  void spotNotReadyMeansCamerasAreUnhealthy() {
    try (var f = new Fixture()) {
      f.status.set(new SpotStatus("test", SpotStatus.State.MISSING_CAMERA, false));
      f.streaming("back", false);
      f.tick(10.04);
      var status = f.vision.getStatus();
      assertThat(status.allRequiredHealthy()).isFalse();
      assertThat(status.usable()).isEqualTo(2);
      assertThat(status.cameras().get("back").usable()).isFalse();
    }
  }

  @Test
  void suppressionDisconnectAndResetInvalidatePendingOrReleasedEstimates() {
    try (var f = new Fixture()) {
      f.estimate("left", 2, 10.01);
      f.tick(10.02);
      assertThat(f.vision.getStatus().queueDepth()).isEqualTo(1);
      f.operator = false;
      f.tick(10.04);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.operator = true;
      f.estimate("left", 2, 10.05);
      f.tick(10.06);
      f.status.close();
      f.tick(10.08);
      assertThat(f.vision.drainMeasurements()).isEmpty();
    }
    try (var f = new Fixture()) {
      f.estimate("left", 2, 10.13);
      f.tick(10.16);
      var value = f.vision.drainMeasurements().get(0);
      f.vision.reset(10.16);
      assertThat(f.vision.eligible(value, 10.16)).isFalse();
      f.estimate("left", 2, 10.15);
      f.tick(10.18);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      assertThat(f.reason("left")).isEqualTo(VisionProcessor.Reason.RESET);
    }
  }

  @Test
  void visionAcceptsLargeInnovationWhenInnovationFilterIsDisabled() {
    try (var f = new Fixture()) {
      f.estimate("left", 2, 10.01);
      f.tick(10.04);
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);

      f.estimatedPose = new Pose2d(0, 4, Rotation2d.kZero);
      f.estimate("left", 2, 10.10);
      f.tick(10.13);

      var results = f.vision.drainMeasurements();
      assertThat(results).hasSize(1);
      assertThat(results.get(0).pose().getX()).isCloseTo(4., offset(1e-9));
      assertThat(results.get(0).innovationM()).isCloseTo(4., offset(1e-9));
    }
  }
}
