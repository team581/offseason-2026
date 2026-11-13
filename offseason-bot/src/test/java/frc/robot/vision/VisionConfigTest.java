package frc.robot.vision;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

final class VisionConfigTest {
  @Test
  void fieldAndFusionPolicyDoNotRequireSpotCameraDefaults() {
    var profile = VisionConfig.profile();
    assertThat(profile.cameras()).isEmpty();
    assertThat(profile.fusionEnabled()).isTrue();
    assertThat(profile.headingEnabled()).isFalse();
    assertThat(profile.field().tags()).hasSize(32);
  }
}
