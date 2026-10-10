package frc.robot.testing;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/** Test report writer. A caller-owned run ID prevents measurements from being overwritten. */
public final class DiagnosticReport {
  public static Path write(
      Path directory, String runId, String configuration, List<DiagnosticRoutine.Result> results)
      throws IOException {
    if (!runId.matches("[a-zA-Z0-9-]+")) {
      throw new IllegalArgumentException("Run ID must contain only letters, digits and hyphens");
    }
    Files.createDirectories(directory);
    String overall =
        !results.isEmpty()
                && results.stream()
                    .allMatch(result -> result.status() == DiagnosticRoutine.Status.PASSED)
            ? "PASSED"
            : results.stream()
                    .anyMatch(result -> result.status() == DiagnosticRoutine.Status.FAILED)
                ? "FAILED"
                : "INCOMPLETE";
    var summary =
        new StringBuilder("# Robot diagnostic regression\n\nRun: ")
            .append(runId)
            .append("\n\nOverall: **")
            .append(overall)
            .append("**")
            .append("\n\nConfiguration: ")
            .append(configuration)
            .append("\n\n| Step | Result | Reason |\n|---|---|---|\n");
    for (var result : results) {
      summary
          .append("| ")
          .append(cell(result.name()))
          .append(" | ")
          .append(result.status())
          .append(" | ")
          .append(cell(result.reason()))
          .append(" |\n");
    }
    Path path = directory.resolve("regression-" + runId + ".md");
    Files.writeString(path, summary, java.nio.file.StandardOpenOption.CREATE_NEW);
    return path;
  }

  private static String cell(String text) {
    return text.replace("|", "\\|").replace('\n', ' ').replace('\r', ' ');
  }

  private DiagnosticReport() {}
}
