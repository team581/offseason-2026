package frc.robot.testing;

import static java.nio.charset.StandardCharsets.UTF_8;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.google.common.collect.ImmutableList;
import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

final class DiagnosticReportTest {
  @TempDir Path directory;

  @Test
  void rejectsPathsAsRunIdentifiers() {
    assertThatThrownBy(
            () -> DiagnosticReport.write(directory, "../escape", "config", ImmutableList.of()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void reportsIncompleteEvidenceAndDoesNotOverwriteEarlierRuns() throws Exception {
    var path =
        DiagnosticReport.write(
            directory,
            "first-run",
            "config",
            ImmutableList.of(
                new DiagnosticRoutine.Result(
                    "score", DiagnosticRoutine.Status.BLOCKED, "No counter | video")));
    assertThat(path).content(UTF_8).contains("INCOMPLETE", "No counter \\| video");
    assertThatThrownBy(
            () -> DiagnosticReport.write(directory, "first-run", "config", ImmutableList.of()))
        .isInstanceOf(FileAlreadyExistsException.class);
    var empty = DiagnosticReport.write(directory, "empty-run", "config", ImmutableList.of());
    assertThat(empty).content(UTF_8).contains("INCOMPLETE");
  }
}
