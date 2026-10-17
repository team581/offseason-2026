package com.team581.simulation;

import com.google.common.collect.ImmutableList;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;

/** Resolves the installed app at process startup, including VS Code external simulation. */
public final class SimulationStudioLauncher {
  public static final String INSTALL_HELP =
      "Simulation Studio is not installed. Download: "
          + "https://github.com/simonstoryparker/simulation-studio/releases/latest . "
          + "Set TEAM581_SIM_APP for a custom installation. "
          + "For the WPILib GUI, rerun with -PwpilibSimGui=true.";

  public static String installedPath() {
    try {
      var found =
          candidates(
                  System.getenv("TEAM581_SIM_APP"),
                  System.getProperty("os.name"),
                  Path.of(System.getProperty("user.home")),
                  System.getenv("LOCALAPPDATA"))
              .stream()
              .filter(SimulationStudioLauncher::isInstalled)
              .findFirst();
      if (found.isPresent()) {
        return found.orElseThrow().toString();
      }
    } catch (RuntimeException exception) {
      System.err.println("Cannot resolve Simulation Studio: " + exception.getMessage());
    }
    System.err.println(INSTALL_HELP);
    return null;
  }

  static List<Path> candidates(String override, String os, Path home, String localAppData) {
    if (override != null && !override.isBlank()) {
      return ImmutableList.of(Path.of(override).toAbsolutePath());
    }
    String normalized = os.toLowerCase(Locale.ROOT);
    if (normalized.contains("mac")) {
      return ImmutableList.of(
          Path.of("/Applications/Simulation Studio.app"),
          home.resolve("Applications/Simulation Studio.app"));
    }
    if (normalized.contains("windows") && localAppData != null) {
      return ImmutableList.of(Path.of(localAppData, "Simulation Studio", "simulation-studio.exe"));
    }
    return ImmutableList.of();
  }

  static boolean isInstalled(Path path) {
    return Files.isRegularFile(
        path.toString().endsWith(".app") ? path.resolve("Contents/MacOS/simulation-studio") : path);
  }

  private SimulationStudioLauncher() {}
}
