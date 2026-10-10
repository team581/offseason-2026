package frc.robot.testing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.nio.file.FileAlreadyExistsException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

final class DiagnosticReportTest {
  @TempDir Path directory;

  @Test
  void rejectsPathsAsRunIdentifiers() {
    assertThatThrownBy(() -> DiagnosticReport.write(directory, "../escape", "config", List.of()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void reportsIncompleteEvidenceAndDoesNotOverwriteEarlierRuns() throws Exception {
    var path =
        DiagnosticReport.write(
            directory,
            "first-run",
            "config",
            List.of(
                new DiagnosticRoutine.Result(
                    "score", DiagnosticRoutine.Status.BLOCKED, "No counter | video")));
    assertThat(Files.readString(path)).contains("INCOMPLETE", "No counter \\| video");
    assertThatThrownBy(() -> DiagnosticReport.write(directory, "first-run", "config", List.of()))
        .isInstanceOf(FileAlreadyExistsException.class);
    var empty = DiagnosticReport.write(directory, "empty-run", "config", List.of());
    assertThat(Files.readString(empty)).contains("INCOMPLETE");
  }
}
