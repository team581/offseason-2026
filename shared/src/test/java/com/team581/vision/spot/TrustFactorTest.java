package com.team581.vision.spot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.data.Offset.offset;

import com.google.common.collect.ImmutableList;
import com.team581.vision.spot.results.VisionMeasurement;
import edu.wpi.first.math.geometry.Pose2d;
import org.junit.jupiter.api.Test;

final class TrustFactorTest {
  private static VisionMeasurement measurement(double time, double innovation) {
    return new VisionMeasurement(
        "front",
        "frame" + time,
        Pose2d.kZero,
        time,
        .1,
        .1,
        1e6,
        ImmutableList.of(1),
        innovation,
        innovation / Math.hypot(.1, .1));
  }

  @Test
  void correlatedEvidenceHasIdenticalOutcomeAcrossBatchesAtCommonObservationTime() {
    var batch = new TrustFactor();
    var split = new TrustFactor();
    batch.reset(10., false);
    split.reset(10., false);
    var strong = measurement(10.01, .01);
    var weak = measurement(10.02, .2);
    batch.updateOdometry(.1, .04);
    batch.ingest(strong);
    batch.ingest(weak);
    split.updateOdometry(.05, .02);
    split.ingest(strong);
    split.updateOdometry(.05, .02);
    split.ingest(weak);
    assertThat(split.get()).isCloseTo(batch.get(), offset(1e-12));
    assertThat(batch.get()).isCloseTo(.2 + .1 + .00375, offset(1e-12));
    assertThat(split.isTrustworthy()).isFalse();
    double before = split.get();
    split.ingest(strong);
    assertThat(split.get()).isEqualTo(before); // stale evidence cannot improve confidence
  }

  @Test
  void seedingIsNotCameraEvidenceAndOdometryUsesPhysicalDistance() {
    var trust = new TrustFactor();
    trust.reset(10., false);
    assertThat(trust.isTrustworthy()).isFalse();
    trust.reset(10., true);
    assertThat(trust.isTrustworthy()).isTrue();
    assertThat(trust.evidenceAge(10.)).isInfinite();
    trust.updateOdometry(0., 120.);
    assertThat(trust.get()).isCloseTo(.254, offset(1e-12)); // sitting still does not age trust
    trust.updateOdometry(2., 0.);
    assertThat(trust.get()).isCloseTo(.354, offset(1e-12));
    assertThatThrownBy(() -> trust.updateOdometry(-1., 0.))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void singleHubTagThatAgreesWithTheEstimateIsTrustworthy() {
    // Field case: left camera, one hub tag ~3.8 m away, 1 cm from the estimate.
    var trust = new TrustFactor(() -> .5);
    trust.reset(10., false);
    double sigma = .015 * 3.8 * 3.8;
    trust.ingest(
        new VisionMeasurement(
            "left", "frame", Pose2d.kZero, 10.01, sigma, sigma, 1e6, ImmutableList.of(1), .01, 0.));
    assertThat(trust.isTrustworthy()).isTrue();
  }
}
