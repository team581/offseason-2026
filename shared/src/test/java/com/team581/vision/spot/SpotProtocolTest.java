package com.team581.vision.spot;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.vision.spot.data.Pose;
import org.junit.jupiter.api.Test;

final class SpotProtocolTest {
  @Test
  void quaternionSignedZeroMatchesPythonFingerprint() {
    for (double component : new double[] {-0.0, -1e-16, 0.0, 1e-16}) {
      assertThat(SpotProtocol.hash(new Pose(0, 0, 0, 1, component, component, component)))
          .isEqualTo("d872e4a9c1306b2ce94accbef5bfd30b9c7e86e6595825f20c6d03992df39c04");
    }
  }
}
