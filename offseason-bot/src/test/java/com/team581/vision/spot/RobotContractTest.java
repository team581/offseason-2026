package com.team581.vision.spot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.PropertyNamingStrategies;
import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.data.*;
import com.team581.vision.spot.results.VisionMeasurement;
import com.team581.vision.spot.structs.FrameResult;
import com.team581.vision.spot.structs.RobotStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.TimestampedRaw;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

final class RobotContractTest {
  private static byte[] fixture(String name) throws IOException {
    String extension = name.equals("manifest") ? ".json" : ".bin";
    try (var input =
        RobotContractTest.class.getResourceAsStream("/spot-robot/" + name + extension)) {
      if (input == null) throw new IOException("fixture missing: " + name);
      return input.readAllBytes();
    }
  }

  private static VisionMeasurement measurement(String source, double timestamp) {
    return new VisionMeasurement(
        source,
        source + timestamp,
        Pose2d.kZero,
        timestamp,
        .1,
        .1,
        1e6,
        ImmutableList.of(1),
        .1,
        1.);
  }

  private static VisionProcessor processor() throws IOException {
    return processor(VisionProcessor.Policy.conservative());
  }

  private static VisionProcessor processor(VisionProcessor.Policy policy) throws IOException {
    return new VisionProcessor(
        profile(),
        timestamp ->
            Optional.of(
                new VisionProcessor.CaptureState(
                    new Pose2d(2, 0, Rotation2d.kZero), 1., 0., 0., 0., 0.)),
        policy);
  }

  private static Profile profile() throws IOException {
    var json =
        new ObjectMapper()
            .setPropertyNamingStrategy(PropertyNamingStrategies.SNAKE_CASE)
            .disable(
                com.fasterxml.jackson.databind.DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
    return json.treeToValue(json.readTree(fixture("manifest")).get("profile"), Profile.class);
  }

  private static VisionIO.Sample recoverySample(String name, int sequence, double time)
      throws IOException {
    return new VisionIO.Sample(
        "front", TestRecords.with(sample(name).frame(), "sequence", (long) sequence), time);
  }

  private static VisionIO.Sample sample(String name) throws IOException {
    var frame =
        FrameResult.struct.unpack(ByteBuffer.wrap(fixture(name)).order(ByteOrder.LITTLE_ENDIAN));
    return new VisionIO.Sample("front", frame, 10.05);
  }

  @CsvSource({".0003125, 4.", ".00125, 4.", ".005, 2.", ".02, 1.", ".08, 1."})
  @ParameterizedTest
  void imageAreaConditioningKeepsUncertaintyWithinItsIntendedBounds(
      double imageAreaFraction, double expectedFactor) throws IOException {
    var frame = sample("multi").frame();
    // Negligible timing uncertainty isolates the image-area factor.
    var policy =
        TestRecords.with(VisionProcessor.Policy.conservative(), "timestampUncertaintyS", 1e-12);
    var baseline =
        processor(policy)
            .process(new VisionIO.Sample("front", frame, 10.05), 10.1)
            .measurement()
            .orElseThrow();
    var adjusted =
        TestRecords.with(
            frame, "primary", TestRecords.with(frame.primary(), "imageArea", imageAreaFraction));
    var result = processor(policy).process(new VisionIO.Sample("front", adjusted, 10.05), 10.1);
    assertThat(result.reason()).isEqualTo(VisionProcessor.Reason.ACCEPTED);
    assertThat(result.measurement().orElseThrow().stdX())
        .isCloseTo(baseline.stdX() * expectedFactor, offset(1e-9));
  }

  @Test
  void immutableMeasurementOwnsTagsAndDeviationVector() {
    var tags = new java.util.ArrayList<Integer>();
    tags.add(1);
    var result =
        new VisionMeasurement(
            "front", "one", new Pose2d(2, 3, Rotation2d.kZero), 10., .1, .2, 1e6, tags, 0., 0.);
    tags.add(2);
    assertThat(result.tagIds()).containsExactly(1);
    result.standardDevs().set(0, 0, 55.);
    assertThat(result.standardDevs().get(0, 0)).isEqualTo(.1);
  }

  @Test
  void ippeSelectionUsesCaptureHeadingAndRejectsUndecidableTranslation() throws IOException {
    assertThat(processor().process(sample("ippe"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.ACCEPTED);
    assertThat(processor().process(sample("ambiguous"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.AMBIGUOUS);
    assertThat(processor().process(sample("empty"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.NO_TAGS);
  }

  @Test
  void malformedStructFieldsCannotBypassValidation() throws IOException {
    var frame = sample("multi").frame();
    for (var invalid :
        ImmutableList.of(
            TestRecords.with(frame, "sequence", 0L),
            TestRecords.with(frame, "candidateCount", 3),
            TestRecords.with(frame, "observedTags", Long.MIN_VALUE),
            TestRecords.with(frame, "totalMs", -1.),
            TestRecords.with(frame, "primary", TestRecords.with(frame.primary(), "method", 9)),
            TestRecords.with(
                frame, "primary", TestRecords.with(frame.primary(), "imageArea", Double.NaN)))) {
      assertThat(processor().process(new VisionIO.Sample("front", invalid, 10.05), 10.1).reason())
          .isEqualTo(VisionProcessor.Reason.MALFORMED);
    }
    for (int source : new int[] {0, 4}) {
      var invalid = TestRecords.with(frame, "timestampSource", source);
      assertThat(processor().process(new VisionIO.Sample("front", invalid, 10.05), 10.1).reason())
          .isEqualTo(VisionProcessor.Reason.TIMESTAMP);
    }
  }

  @Test
  void missingCaptureHistoryNeverSubstitutesCurrentPose() throws IOException {
    var processor =
        new VisionProcessor(
            profile(), timestamp -> Optional.empty(), VisionProcessor.Policy.conservative());
    assertThat(processor.process(sample("multi"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.HISTORY);
  }

  @Test
  void olderCaptureIsRejectedEvenWithAHigherSequence() throws IOException {
    var sample = sample("multi");
    var processor = processor();
    assertThat(processor.process(sample, 10.1).reason()).isEqualTo(VisionProcessor.Reason.ACCEPTED);
    var replacement =
        TestRecords.with(sample.frame(), "captureHigh", sample.frame().captureHigh() ^ 1L);
    assertThat(processor.process(new VisionIO.Sample("front", replacement, 10.06), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.ACCEPTED);
    var retired = TestRecords.with(sample.frame(), "sequence", 2L);
    assertThat(processor.process(new VisionIO.Sample("front", retired, 10.05), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.DUPLICATE);
  }

  @Test
  void pythonAndJavaAgreeOnPacketsAndCoordinateUnits() throws IOException {
    for (String name : ImmutableList.of("empty", "multi", "ippe", "ambiguous")) {
      assertThat(fixture(name)).hasSize(FrameResult.struct.getSize());
      assertThat(sample(name).frame().sequence()).isEqualTo(1);
    }
    var accepted = processor().process(sample("multi"), 10.1);
    assertThat(accepted.reason()).isEqualTo(VisionProcessor.Reason.ACCEPTED);
    var measurement = accepted.measurement().orElseThrow();
    assertThat(measurement.pose().getX()).isCloseTo(2., offset(1e-12));
    assertThat(measurement.pose().getY()).isCloseTo(0., offset(1e-12));
    assertThat(measurement.timestamp()).isEqualTo(10.05);
    assertThat(measurement.stdX()).isPositive();
    assertThat(measurement.stdHeading()).isEqualTo(1e6);
  }

  @Test
  void pythonPolicyStructUsesTheSameFieldLayoutAndGeneration() throws IOException {
    var bytes = fixture("policy-state");
    assertThat(bytes).hasSize(RobotStatus.struct.getSize());
    var state = RobotStatus.struct.unpack(ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN));
    assertThat(state.selectionGeneration()).isEqualTo(3);
    assertThat(state.allowedTags()).isEqualTo(3);
    var frame =
        FrameResult.struct.unpack(
            ByteBuffer.wrap(fixture("policy-multi")).order(ByteOrder.LITTLE_ENDIAN));
    assertThat(frame.tagSelectionGeneration()).isEqualTo(3);
    assertThat(frame.primary().tagMask()).isEqualTo(3);
  }

  @CsvSource({"false, 0, multi", "true, 0, ippe", "true, 1, multi"})
  @ParameterizedTest
  void recoveryDoesNotBypassFreshEvidenceMotionOrSingleTagSafeguards(
      boolean recoveryAllowed, double speed, String name) throws IOException {
    var processor =
        new VisionProcessor(
            profile(),
            time ->
                Optional.of(
                    new VisionProcessor.CaptureState(
                        new Pose2d(6, 0, Rotation2d.kZero), speed, 0, 0, 0, 0)),
            VisionProcessor.Policy::conservative,
            () -> recoveryAllowed);
    for (int i = 1; i <= 6; i++) {
      double time = 10. + .06 * i;
      assertThat(processor.process(recoverySample(name, i, time), time + .03).reason())
          .isEqualTo(VisionProcessor.Reason.INNOVATION);
    }
  }

  @Test
  void recoveryNeedsElapsedCaptureTimeAndCannotReuseStaleEvidence() throws IOException {
    var processor =
        new VisionProcessor(
            profile(),
            time ->
                Optional.of(
                    new VisionProcessor.CaptureState(
                        new Pose2d(6, 0, Rotation2d.kZero), 0, 0, 0, 0, 0)),
            VisionProcessor.Policy::conservative,
            () -> true);
    for (int i = 1; i <= 5; i++) {
      double time = 10 + .001 * i;
      assertThat(processor.process(recoverySample("multi", i, time), time + .03).reason())
          .isEqualTo(VisionProcessor.Reason.INNOVATION);
    }
    assertThat(processor.process(recoverySample("multi", 6, 10.70), 10.73).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    assertThat(processor.process(recoverySample("multi", 7, 10.76), 10.79).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    assertThat(processor.process(recoverySample("multi", 8, 10.82), 10.85).reason())
        .isEqualTo(VisionProcessor.Reason.ACCEPTED);
  }

  @Test
  void recoveryRequiresAgreementAndEvidenceDoesNotSurviveResets() throws IOException {
    var pose =
        new java.util.concurrent.atomic.AtomicReference<>(new Pose2d(6, 0, Rotation2d.kZero));
    var processor =
        new VisionProcessor(
            profile(),
            time -> Optional.of(new VisionProcessor.CaptureState(pose.get(), 0, 0, 0, 0, 0)),
            VisionProcessor.Policy::conservative,
            () -> true);
    for (int i = 1; i <= 4; i++) {
      pose.set(new Pose2d(i % 2 == 0 ? 7 : 6, 0, Rotation2d.kZero));
      double time = 10 + .06 * i;
      assertThat(processor.process(recoverySample("multi", i, time), time + .03).reason())
          .isEqualTo(VisionProcessor.Reason.INNOVATION);
    }
    for (int i = 5; i <= 6; i++) {
      double time = 10 + .06 * i;
      processor.process(recoverySample("multi", i, time), time + .03);
      processor.reset(time);
    }
    assertThat(processor.process(recoverySample("multi", 7, 10.42), 10.45).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    processor.process(recoverySample("multi", 8, 10.48), 10.51);
    processor.clearRecoveryEvidence();
    assertThat(processor.process(recoverySample("multi", 9, 10.54), 10.57).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    processor.process(recoverySample("multi", 10, 10.60), 10.63);
    processor.newGeneration("replacement");
    assertThat(processor.process(recoverySample("multi", 11, 10.66), 10.69).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
  }

  @Test
  void reorderWindowSpansLoopsAndDoesNotWaitForOfflineCamera() {
    var queue = new MeasurementQueue(.02, 8);
    queue.add(measurement("fast", 10.));
    assertThat(queue.release(10.01, 32)).isEmpty();
    queue.add(measurement("slow", 9.99));
    assertThat(queue.release(10.03, 32).stream().map(VisionMeasurement::timestamp))
        .containsExactly(9.99, 10.);
    assertThat(queue.add(measurement("late", 9.98))).isFalse();
    assertThat(queue.lateCount()).isEqualTo(1);
    queue.reset(11.);
    assertThat(queue.add(measurement("pre-reset", 10.99))).isFalse();
  }

  @Test
  void simultaneousCamerasCannotImproveTrustByCameraCount() {
    var result = ImmutableList.of(measurement("a", 10.), measurement("b", 10.));
    var trust = new TrustFactor();
    trust.ingest(result.get(0));
    double before = trust.get();
    result.forEach(trust::ingest);
    assertThat(trust.get()).isEqualTo(before);
    trust.updateOdometry(1., .02, false);
    assertThat(trust.get()).isGreaterThan(before);
  }

  @Test
  void staleFutureMalformedGenerationDuplicateAndResetPacketsFailClosed() throws IOException {
    var processor = processor();
    assertThat(processor.process(sample("multi"), 10.4).reason())
        .isEqualTo(VisionProcessor.Reason.STALE);
    assertThat(processor.process(sample("multi"), 10.).reason())
        .isEqualTo(VisionProcessor.Reason.FUTURE);
    assertThat(processor.process(sample("multi"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.ACCEPTED);
    assertThat(processor.process(sample("multi"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.DUPLICATE);
    processor.reset(10.06);
    assertThat(processor.process(sample("multi"), 10.1).reason())
        .isEqualTo(VisionProcessor.Reason.RESET);
  }

  @Test
  void stationaryConsistentMultiTagEvidenceCanRecoverAnIncorrectInitialPose() throws IOException {
    var processor =
        new VisionProcessor(
            profile(),
            time ->
                Optional.of(
                    new VisionProcessor.CaptureState(
                        new Pose2d(6, 0, Rotation2d.kZero), 0, 0, 0, 0, 0)),
            VisionProcessor.Policy::conservative,
            () -> true);
    assertThat(processor.process(recoverySample("multi", 1, 10.01), 10.04).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    assertThat(processor.process(recoverySample("multi", 2, 10.07), 10.10).reason())
        .isEqualTo(VisionProcessor.Reason.INNOVATION);
    var recovered = processor.process(recoverySample("multi", 3, 10.13), 10.16);
    assertThat(recovered.reason()).isEqualTo(VisionProcessor.Reason.ACCEPTED);
    assertThat(recovered.measurement().orElseThrow().pose().getX()).isCloseTo(2., offset(1e-9));
    // A large correction remains visible to trust tracking until the estimator converges.
    assertThat(recovered.measurement().orElseThrow().innovationM()).isCloseTo(4., offset(1e-9));
  }

  @Test
  void timestampDomainsHandleSentinels() {
    var remote = new TimestampedRaw(10_050_000, 10_050_000, new byte[0]);
    assertThat(SpotVisionIO.captureTimestamp(remote).orElseThrow()).isEqualTo(10.05);
    for (long sentinel : ImmutableList.of(0L, 1L)) {
      assertThat(
              SpotVisionIO.captureTimestamp(new TimestampedRaw(10_050_000, sentinel, new byte[0]))
                  .orElseThrow())
          .isEqualTo(10.05);
    }
    assertThat(
            SpotVisionIO.captureTimestamp(new TimestampedRaw(10_050_000, 99_000_000, new byte[0]))
                .orElseThrow())
        .isEqualTo(99.);
  }

  @Test
  void transformedCameraMountUsesNwuAndInverseComposition() {
    var robot = new Pose3d(2, 3, .1, new Rotation3d(0, 0, Math.PI / 2));
    var mount = new Transform3d(new Translation3d(.2, -.3, .5), new Rotation3d(.1, -.2, .3));
    var recovered =
        SpotProtocol.pose(SpotProtocol.pose(robot.transformBy(mount))).transformBy(mount.inverse());
    assertThat(recovered.getTranslation().getDistance(robot.getTranslation())).isLessThan(1e-12);
    assertThat(recovered.getRotation().minus(robot.getRotation()).getAngle()).isLessThan(1e-12);
  }
}
