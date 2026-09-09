package com.team581.util.profiling;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class DiagnosticCadenceTest {
  @AfterEach
  void disableCadence() {
    DiagnosticCadence.setEnabled(false);
  }

  @Test
  void disabledCadenceAlwaysLogs() {
    DiagnosticCadence.setEnabled(false);
    for (int i = 0; i < 20; i++) {
      DiagnosticCadence.beginLoop();
      assertThat(DiagnosticCadence.shouldLogRoutine()).isTrue();
      assertThat(DiagnosticCadence.shouldLogHeavy()).isTrue();
    }
  }

  @Test
  void enabledCadenceLogsImmediatelyThenAtConfiguredIntervals() {
    DiagnosticCadence.setEnabled(true);
    for (int loop = 1; loop <= 20; loop++) {
      DiagnosticCadence.beginLoop();
      assertThat(DiagnosticCadence.shouldLogRoutine()).isEqualTo(loop % 5 == 1);
      assertThat(DiagnosticCadence.shouldLogHeavy()).isEqualTo(loop % 10 == 1);
    }
  }
}
