package com.team581.vision.spot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.data.*;
import com.team581.vision.spot.results.VisionMeasurement;
import com.team581.vision.spot.structs.FrameResult;
import com.team581.vision.spot.structs.PoseEstimate;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class VisionIntegrationTest {
  static final class FakeIO implements VisionIO {
    String generation = "initial";
    Profile profile;
    TagSelection selection;
    boolean available = true;
    int reads, closed;
    Runnable duringRead = () -> {};
    final List<Sample> samples = new ArrayList<>();

    @Override
    public void close() {
      closed++;
    }

    @Override
    public Optional<Profile> configuration() {
      return Optional.ofNullable(profile);
    }

    @Override
    public Inputs readInputs(double now) {
      reads++;
      duringRead.run();
      var result =
          new Inputs(new Health(available, available ? "Ready" : "Offline", generation), samples);
      samples.clear();
      return result;
    }

    @Override
    public void setTagSelection(TagSelection value) {
      selection = value;
    }
  }

  /** These mounts and optical modes are deliberately synthetic, never production settings. */
  static final class Fixture implements AutoCloseable {
    double time = 10.;
    boolean operator = true;
    Optional<Alliance> alliance = Optional.of(Alliance.Red);
    final Pose2d robot = new Pose2d(4, 4, Rotation2d.kZero);
    Pose2d estimatedPose = robot;
    final FakeIO io = new FakeIO();
    final Profile profile;
    final Vision vision;
    long sequence;
    final java.util.Map<String, Transform3d> mounts = new java.util.HashMap<>();

    Fixture(boolean fusion) {
      var fieldTags = new ArrayList<FieldTag>();
      var field = new FieldLayout("blue-wall-right", 16, 8, ImmutableList.of());
      for (int id : Set.of(1, 2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27)) {
        int direction =
            switch (id) {
              case 2, 8, 18, 24 -> 0;
              case 3, 9, 19, 25 -> 1;
              case 5, 11, 21, 27 -> 3;
              default -> 2;
            };
        double x = new double[] {4, 4, 7, 1}[direction], y = new double[] {7, 1, 4, 4}[direction];
        fieldTags.add(new FieldTag(id, new Pose(x, y, 1, 1, 0.0, 0.0, 0.0), .1651));
      }
      field = TestRecords.with(field, "tags", fieldTags);
      var cameraDefinitions = new ArrayList<CameraPolicy>();
      var builder = new Profile(ImmutableList.of(), field, fusion, false, .25, .005, .02);
      int index = 0;
      for (String name : SYNTHETIC_CAMERA_NAMES) {
        double yaw = new double[] {Math.PI / 2, -Math.PI / 2, 0, Math.PI}[index];
        var mount =
            new Transform3d(new Translation3d(.1 * index, .1, .5), new Rotation3d(0, 0, yaw));
        index++;
        mounts.put(name, mount);
        cameraDefinitions.add(new CameraPolicy(name, true, true));
      }
      profile = TestRecords.with(builder, "cameras", cameraDefinitions);
      io.profile = profile;
      vision =
          new Vision(
              io,
              profile,
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
              () -> operator,
              () -> alliance);
      vision.collectInputs();
    }

    @Override
    public void close() {
      vision.close();
    }

    void frame(String camera, int tag, double capture) {
      frame(camera, tag, capture, io.selection.generation());
    }

    void frame(String camera, int tag, double capture, long selectionGeneration) {
      var mount = mounts.get(camera);
      var empty = new PoseEstimate(Pose3d.kZero, 0, 0, 0, 0, 0, 0, 0);
      var primary = empty;
      if (tag > 0) {
        int companion =
            switch (tag) {
              case 1 -> 10;
              case 2 -> 8;
              case 3 -> 9;
              case 4 -> 10;
              case 5 -> 11;
              case 18 -> 24;
              case 19 -> 25;
              case 20 -> 26;
              case 21 -> 27;
              default -> 1;
            };
        primary =
            new PoseEstimate(
                new Pose3d(robot).transformBy(mount),
                1,
                SpotProtocol.tagMask(ImmutableList.of(tag, companion), profile.field()),
                .1,
                2,
                .02,
                .1,
                0.);
      }
      var frame =
          new FrameResult(
              1L,
              2L,
              3L,
              4L,
              ++sequence,
              selectionGeneration,
              0L,
              1,
              primary.tagMask(),
              tag > 0 ? 1 : 0,
              mount,
              true,
              primary,
              empty,
              0.,
              0.,
              0.,
              0.);
      io.samples.add(new VisionIO.Sample(camera, frame, capture));
    }

    VisionProcessor.Reason reason(String camera) {
      return VisionProcessor.Reason.valueOf(vision.getStatus().cameras().get(camera).reason());
    }

    void tick(double now) {
      time = now;
      vision.collectInputs();
      vision.robotPeriodic();
    }
  }

  private static final List<String> SYNTHETIC_CAMERA_NAMES =
      ImmutableList.of("left", "right", "front", "back");

  @BeforeAll
  static void hal() {
    HAL.initialize(500, 0);
  }

  @Test
  void disabledCameraCannotContributeWhileOtherCamerasRemainAvailable() {
    try (var f = new Fixture(true)) {
      var policies =
          f.profile.cameras().stream()
              .map(c -> new CameraPolicy(c.name(), c.required(), !c.name().equals("front")))
              .toList();
      f.io.profile = TestRecords.with(f.profile, "cameras", policies);
      f.frame("front", 1, 10.01);
      f.frame("left", 2, 10.01);
      f.tick(10.04);
      assertThat(f.vision.drainMeasurements())
          .extracting(VisionMeasurement::source)
          .containsExactly("left");
    }
  }

  @Test
  void fourStaticCamerasRecoverSamePoseAndDrainOnce() {
    try (var f = new Fixture(true)) {
      assertThat(f.profile.cameras().stream().map(CameraPolicy::name))
          .containsExactly("left", "right", "front", "back");
      for (int i = 0; i < 4; i++)
        f.frame(SYNTHETIC_CAMERA_NAMES.get(i), new int[] {2, 3, 1, 5}[i], 10.01);
      f.tick(10.04);
      var batch = f.vision.drainMeasurements();
      assertThat(batch).hasSize(4);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      for (var value : batch) {
        assertThat(value.pose().getX()).isCloseTo(4., offset(1e-10));
        assertThat(value.pose().getY()).isCloseTo(4., offset(1e-10));
        assertThat(value.timestamp()).isEqualTo(10.01);
        assertThat(value.stdHeading()).isEqualTo(2e6);
        assertThat(f.vision.eligible(value, 10.04)).isTrue();
        f.vision.measurementFused(value);
        f.vision.measurementFused(value);
      }
      assertThat(f.vision.getStatus().cameras().values())
          .allSatisfy(c -> assertThat(c.lastFused()).isEqualTo(10.01));
      int reads = f.io.reads;
      assertThat(f.vision.getStatus().allRequiredHealthy()).isTrue();
      f.vision.getStatus();
      f.vision.seeingTag();
      assertThat(f.io.reads).isEqualTo(reads);
      f.tick(10.06);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.vision.close();
      f.vision.close();
      assertThat(f.io.closed).isEqualTo(1);
    }
  }

  @Test
  void hubAcquisitionHandoffLossAndRepeatedRequestsUseOneGlobalPolicy() {
    try (var f = new Fixture(false)) {
      f.vision.hubTagsRequest();
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      f.frame("back", 21, 10.01);
      f.tick(10.04);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      f.frame("left", 2, 10.05);
      f.tick(10.08);
      assertThat(f.vision.getState()).isEqualTo(VisionState.HUB_TAGS);
      assertThat(f.io.selection.allowedTagIds()).containsExactly(2, 3, 4, 5, 8, 9, 10, 11);
      long generation = f.io.selection.generation();
      f.vision.hubTagsRequest();
      f.tick(10.3);
      assertThat(f.io.selection.generation()).isEqualTo(generation);
      assertThat(f.vision.getState()).isEqualTo(VisionState.HUB_TAGS);
      f.frame("right", 3, 10.32);
      f.tick(10.34);
      f.tick(10.6);
      assertThat(f.vision.getState()).isEqualTo(VisionState.HUB_TAGS);
      f.tick(10.85);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      f.vision.tagsRequest();
      assertThat(f.vision.getState()).isEqualTo(VisionState.TAGS);
      assertThat(f.vision.drainMeasurements()).isEmpty(); // shadow cannot fuse
    }
  }

  @Test
  void inputCollectionDelayDoesNotMakeArrivingFramesAppearFuture() {
    try (var f = new Fixture(false)) {
      f.io.duringRead = () -> f.time += .030;
      f.frame("front", 0, f.time + .010);
      f.vision.collectInputs();
      assertThat(f.reason("front")).isEqualTo(VisionProcessor.Reason.NO_TAGS);
      // A genuinely future timestamp still fails the unchanged tolerance gate.
      f.frame("front", 0, f.time + .100);
      f.vision.collectInputs();
      assertThat(f.reason("front")).isEqualTo(VisionProcessor.Reason.FUTURE);
    }
  }

  @Test
  void oldPolicyExcludedIdsAndRejectedObservationsCannotAcquireHub() {
    try (var f = new Fixture(false)) {
      f.vision.hubTagsRequest();
      f.frame("left", 2, 9.5);
      f.tick(10.04);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      f.frame("front", 4, 10.05);
      f.tick(10.08);
      f.frame("front", 20, 10.09); // current generation with forbidden candidate
      f.frame("left", 2, 10.09, 1); // in-flight previous policy
      f.tick(10.12);
      assertThat(f.reason("front")).isEqualTo(VisionProcessor.Reason.TAG_POLICY);
      assertThat(f.reason("left")).isEqualTo(VisionProcessor.Reason.TAG_POLICY);
      f.tick(10.59);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
    }
  }

  @Test
  void requiredFailureOptionalFailureAndOperatorOverridesAreHonest() {
    try (var f = new Fixture(true)) {
      var health = new frc.robot.health.HealthManager(f.vision, false);
      health.setPoseTrustworthySupplier(() -> true);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
      f.frame("front", 1, 10.01);
      f.tick(10.04);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse(); // accepted is not fused
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();
      assertThat(health.isAllCamerasHealthy()).isFalse();
      assertThat(health.isFuelDetectionHealthy()).isFalse();
      health.setPoseTrustworthySupplier(() -> false);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
      health.setPoseTrustworthySupplier(() -> true);
      health.beforePeriodic();
      try (var use =
              edu.wpi.first.networktables.NetworkTableInstance.getDefault()
                  .getBooleanTopic("/DSOptions/UseVision")
                  .publish();
          var pit =
              edu.wpi.first.networktables.NetworkTableInstance.getDefault()
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
      f.tick(10.6);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
      f.frame("back", 5, 10.61);
      f.tick(10.64);
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();
    }
  }

  @Test
  void shadowFusionAndResetCannotClaimLocalizationReadiness() {
    try (var shadow = new Fixture(false)) {
      shadow.frame("front", 1, 10.01);
      shadow.tick(10.04);
      assertThat(shadow.vision.getStatus().usable()).isPositive();
      assertThat(shadow.vision.isFusionEnabled()).isFalse();
      assertThat(shadow.vision.hasRecentFusedMeasurement()).isFalse();
    }
    try (var active = new Fixture(true)) {
      active.frame("front", 1, 10.01);
      active.tick(10.04);
      active.vision.drainMeasurements().forEach(active.vision::measurementFused);
      assertThat(active.vision.hasRecentFusedMeasurement()).isTrue();
      active.vision.reset(10.04);
      assertThat(active.vision.hasRecentFusedMeasurement()).isFalse();
    }
  }

  @Test
  void statusSnapshotsStayConsistentAcrossFusionAndResetWithoutPolling() {
    try (var f = new Fixture(true)) {
      var waiting = f.vision.getStatus();
      assertThat(waiting.configured()).isEqualTo(4);
      assertThat(waiting.required()).isEqualTo(4);
      assertThat(waiting.usable()).isEqualTo(0);
      f.frame("front", 1, 10.01);
      f.tick(10.04);
      var ready = f.vision.getStatus();
      assertThat(ready.usable()).isEqualTo(1);
      assertThat(ready.selectionPending()).isFalse();
      assertThat(waiting.cameras().get("front").usable()).isFalse();
      int reads = f.io.reads;
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);
      assertThat(f.vision.getStatus().cameras().get("front").lastFused()).isEqualTo(10.01);
      assertThat(ready.cameras().get("front").lastFused()).isNegative().isInfinite();
      f.vision.reset(10.04);
      var reset = f.vision.getStatus();
      assertThat(reset.usable()).isEqualTo(0);
      assertThat(reset.selectionPending()).isTrue();
      assertThat(reset.allRequiredHealthy()).isFalse();
      assertThat(reset.queueDepth()).isEqualTo(0);
      assertThat(ready.cameras().get("front").usable()).isTrue();
      assertThat(f.io.reads).isEqualTo(reads);
    }
  }

  @Test
  void suppressionDisconnectGenerationAndResetInvalidatePendingOrReleasedFrames() {
    try (var f = new Fixture(true)) {
      f.frame("front", 1, 10.01);
      f.tick(10.02);
      assertThat(f.vision.getStatus().queueDepth()).isEqualTo(1);
      f.operator = false;
      f.tick(10.04);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.operator = true;
      f.frame("front", 1, 10.05);
      f.tick(10.06);
      f.io.available = false;
      f.tick(10.08);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.io.available = true;
      f.frame("front", 1, 10.09);
      f.tick(10.10);
      f.io.generation = "new-runtime";
      f.tick(10.12);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      f.frame("front", 1, 10.13);
      f.tick(10.16);
      var value = f.vision.drainMeasurements().get(0);
      f.vision.reset(10.16);
      assertThat(f.vision.eligible(value, 10.16)).isFalse();
      f.vision.measurementFused(value);
      assertThat(f.vision.getStatus().cameras().get("front").lastFused())
          .isNotEqualTo(value.timestamp());
      f.frame("front", 1, 10.15);
      f.tick(10.18);
      assertThat(f.vision.drainMeasurements()).isEmpty();
      assertThat(f.reason("front")).isEqualTo(VisionProcessor.Reason.RESET);
    }
  }

  @Test
  void tagFilterChangesPreserveFusedHealthButDiscardPendingFramesAndStillExpire() {
    try (var f = new Fixture(true)) {
      var health = new frc.robot.health.HealthManager(f.vision, false);
      health.setPoseTrustworthySupplier(() -> true);
      f.frame("left", 2, 10.01);
      f.tick(10.04);
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();

      f.vision.hubTagsRequest();
      f.vision.robotPeriodic();
      assertThat(f.vision.getState()).isEqualTo(VisionState.HUB_TAGS);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();

      f.frame("left", 2, 10.05);
      f.tick(10.08);
      var pending = f.vision.drainMeasurements();
      assertThat(pending).isNotEmpty();
      f.vision.tagsRequest();
      pending.forEach(value -> assertThat(f.vision.eligible(value, f.time)).isFalse());
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isTrue();
      assertThat(f.vision.getStatus().cameras().get("left").lastFused()).isEqualTo(10.01);

      f.tick(10.30);
      health.beforePeriodic();
      assertThat(health.isLocalizationHealthy()).isFalse();
    }
  }

  @Test
  void unknownAllianceStaysBroadAndChangingAllianceReacquires() {
    try (var f = new Fixture(false)) {
      f.alliance = Optional.empty();
      f.vision.hubTagsRequest();
      f.frame("front", 4, 10.01);
      f.tick(10.04);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      assertThat(f.io.selection.allowedTagIds()).contains(1, 18, 27);
      f.alliance = Optional.of(Alliance.Blue);
      f.tick(10.06);
      f.frame("back", 21, 10.07);
      f.tick(10.10);
      assertThat(f.io.selection.allowedTagIds()).containsExactly(18, 19, 20, 21, 24, 25, 26, 27);
      f.alliance = Optional.of(Alliance.Red);
      f.tick(10.12);
      assertThat(f.vision.getState()).isEqualTo(VisionState.WAITING_FOR_HUB_TAGS);
      assertThat(f.io.selection.allowedTagIds()).contains(1, 18, 2);
    }
  }

  @Test
  void visionEnablesGuardedRecoveryAfterFusedEvidenceExpires() {
    try (var f = new Fixture(true)) {
      f.frame("front", 1, 10.01);
      f.tick(10.04);
      f.vision.drainMeasurements().forEach(f.vision::measurementFused);
      assertThat(f.vision.hasRecentFusedMeasurement()).isTrue();

      f.estimatedPose = new Pose2d(0, 4, Rotation2d.kZero);
      for (int i = 1; i <= 3; i++) {
        double time = 10.04 + .06 * i;
        f.frame("front", 1, time);
        f.tick(time + .03);
        assertThat(f.vision.drainMeasurements()).isEmpty();
      }
      // Fresh fusion evidence has now expired. Three consistent observations may recover.
      for (int i = 1; i <= 3; i++) {
        double time = 10.30 + .06 * i;
        f.frame("front", 1, time);
        f.tick(time + .03);
        var results = f.vision.drainMeasurements();
        if (i < 3) {
          assertThat(results).isEmpty();
        } else {
          assertThat(results).hasSize(1);
          assertThat(results.get(0).pose().getX()).isCloseTo(4., offset(1e-9));
          results.forEach(f.vision::measurementFused);
          assertThat(f.vision.hasRecentFusedMeasurement()).isTrue();
        }
      }
    }
  }

  @Test
  void zeroTagFramesAreHealthyButPoseTrustIsIndependent() {
    try (var f = new Fixture(false)) {
      for (var name : SYNTHETIC_CAMERA_NAMES) f.frame(name, 0, 10.01);
      f.tick(10.04);
      assertThat(f.vision.getStatus().usable()).isEqualTo(4);
      assertThat(f.vision.getStatus().allRequiredHealthy()).isTrue();
      assertThat(f.vision.seeingTag()).isFalse();
      f.tick(10.53);
      assertThat(f.vision.getStatus().usable()).isEqualTo(0);
      assertThat(f.vision.getStatus().allRequiredHealthy()).isFalse();
      assertThat(f.vision.hasRecentFusedMeasurement()).isFalse();
    }
  }
}
