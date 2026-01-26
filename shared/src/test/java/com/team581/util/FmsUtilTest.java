package com.team581.util;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class FmsUtilTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void hubIsActiveInAutoTest() {
    var time = 5.0;
    // 0.0 - 20.0
    assertThat(FmsUtil.isHubActive(time)).isTrue();
  }

  @Test
  void hubIsActiveInShift2Test() {
    var time = 60.0;
    // 55.0 - 80.0
    assertThat(FmsUtil.isHubActive(time)).isTrue();
  }

  @Test
  void hubIsActiveInTransitionShiftTest() {
    var time = 25.0;
    // 20.0 - 30.0
    assertThat(FmsUtil.isHubActive(time)).isTrue();
  }

  @Test
  void hubIsNotActiveInShift1Test() {
    var time = 35.0;
    // 35.0 - 55.0
    assertThat(FmsUtil.isHubActive(time)).isFalse();
  }
}
