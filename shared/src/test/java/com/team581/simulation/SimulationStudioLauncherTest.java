package com.team581.simulation;

import static org.assertj.core.api.Assertions.assertThat;

import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

final class SimulationStudioLauncherTest {
  @TempDir Path directory;

  @Test
  void discoversMacAndWindowsInstallations() {
    assertThat(SimulationStudioLauncher.candidates(null, "Mac OS X", directory, null))
        .containsExactly(
            Path.of("/Applications/Simulation Studio.app"),
            directory.resolve("Applications/Simulation Studio.app"));
    assertThat(
            SimulationStudioLauncher.candidates(
                null, "Windows 11", directory, directory.toString()))
        .containsExactly(directory.resolve("Simulation Studio/simulation-studio.exe"));
    assertThat(SimulationStudioLauncher.candidates(null, "Linux", directory, null)).isEmpty();
  }

  @Test
  void explicitPathTakesPrecedenceWithoutSilentlyFallingBack() {
    var paths =
        SimulationStudioLauncher.candidates(
            directory.resolve("Custom Studio.app").toString(), "Mac OS X", directory, null);
    assertThat(paths).containsExactly(directory.resolve("Custom Studio.app"));
    assertThat(SimulationStudioLauncher.isInstalled(paths.get(0))).isFalse();
  }

  @Test
  void macBundleRequiresItsExecutable() throws Exception {
    var bundle = directory.resolve("Simulation Studio.app");
    Files.createDirectories(bundle.resolve("Contents/MacOS"));
    assertThat(SimulationStudioLauncher.isInstalled(bundle)).isFalse();
    Files.createFile(bundle.resolve("Contents/MacOS/simulation-studio"));
    assertThat(SimulationStudioLauncher.isInstalled(bundle)).isTrue();
  }
}
