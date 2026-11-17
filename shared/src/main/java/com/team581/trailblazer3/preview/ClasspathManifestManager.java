package com.team581.trailblazer3.preview;

import static java.util.Objects.requireNonNull;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HexFormat;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

/**
 * Manages the cached Gradle classpath manifest ({@code
 * .gradle/trailblazer-preview/classpath.manifest}) with SHA-256 build-file invalidation and
 * automatic Gradle refresh.
 */
public final class ClasspathManifestManager {
  /** Serialized classpath manifest record. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record ClasspathManifest(
      String buildFilesHash,
      List<String> classpathEntries,
      String jniLibraryPath,
      List<String> sourceRoots) {
    public ClasspathManifest {
      requireNonNull(buildFilesHash);
      classpathEntries = List.copyOf(classpathEntries);
      requireNonNull(jniLibraryPath);
      sourceRoots = List.copyOf(sourceRoots);
    }

    public String toClasspathString() {
      return String.join(File.pathSeparator, classpathEntries);
    }

    public String toSourcepathString() {
      return String.join(File.pathSeparator, sourceRoots);
    }
  }

  public static final String MANIFEST_RELATIVE_PATH =
      ".gradle/trailblazer-preview/classpath.manifest";

  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final List<String> TRACKED_BUILD_FILES =
      List.of(
          "build.gradle",
          "settings.gradle",
          "gradle.properties",
          "shared/build.gradle",
          "comp-bot/build.gradle",
          "offseason-bot/build.gradle",
          "new-member-bot/build.gradle");

  /** Computes the deterministic SHA-256 hash of build scripts and vendordep JSONs. */
  public static String computeBuildFilesHash(Path workspaceRoot) {
    requireNonNull(workspaceRoot);
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      List<Path> filesToHash = new ArrayList<>();
      for (String rel : TRACKED_BUILD_FILES) {
        Path p = workspaceRoot.resolve(rel);
        if (Files.isRegularFile(p)) {
          filesToHash.add(p);
        }
      }
      for (String sub : List.of(".", "shared", "comp-bot", "offseason-bot", "new-member-bot")) {
        Path vendorDir = workspaceRoot.resolve(sub).resolve("vendordeps");
        if (Files.isDirectory(vendorDir)) {
          try (Stream<Path> stream = Files.list(vendorDir)) {
            stream
                .filter(p -> Files.isRegularFile(p) && p.toString().endsWith(".json"))
                .sorted(Comparator.comparing(Path::toString))
                .forEach(filesToHash::add);
          }
        }
      }
      for (Path file : filesToHash) {
        String relPath = workspaceRoot.relativize(file).toString().replace('\\', '/');
        digest.update(relPath.getBytes(StandardCharsets.UTF_8));
        digest.update((byte) 0);
        digest.update(Files.readAllBytes(file));
        digest.update((byte) 0);
      }
      return HexFormat.of().formatHex(digest.digest());
    } catch (NoSuchAlgorithmException | IOException e) {
      throw new IllegalStateException("Failed to hash workspace build files", e);
    }
  }

  /**
   * Loads the cached classpath manifest if it exists, matches the current build-files SHA-256 hash,
   * and all referenced classpath entries exist on disk.
   */
  public static Optional<ClasspathManifest> loadIfValid(Path workspaceRoot) {
    requireNonNull(workspaceRoot);
    Path file = manifestPath(workspaceRoot);
    if (!Files.isRegularFile(file)) {
      return Optional.empty();
    }
    try {
      ClasspathManifest manifest = MAPPER.readValue(file.toFile(), ClasspathManifest.class);
      String currentHash = computeBuildFilesHash(workspaceRoot);
      if (!currentHash.equals(manifest.buildFilesHash())) {
        return Optional.empty();
      }
      if (manifest.classpathEntries().isEmpty()) {
        return Optional.empty();
      }
      boolean hasExistingEntry = false;
      for (String entry : manifest.classpathEntries()) {
        if (Files.exists(Path.of(entry))) {
          hasExistingEntry = true;
        } else if (entry.endsWith(".jar")) {
          return Optional.empty();
        }
      }
      return hasExistingEntry ? Optional.of(manifest) : Optional.empty();
    } catch (IOException e) {
      return Optional.empty();
    }
  }

  /**
   * Loads the cached manifest when valid, or refreshes it by invoking {@code ./gradlew
   * writePreviewClasspathManifest} when missing or invalidated by build-file changes.
   */
  public static ClasspathManifest loadOrRefresh(Path workspaceRoot) {
    Optional<ClasspathManifest> cached = loadIfValid(workspaceRoot);
    if (cached.isPresent()) {
      return cached.orElseThrow();
    }
    refreshViaGradle(workspaceRoot);
    return loadIfValid(workspaceRoot)
        .orElseThrow(() ->
            new IllegalStateException(
                "Gradle writePreviewClasspathManifest did not produce a valid manifest"));
  }

  /** Returns the manifest file path under the workspace root. */
  public static Path manifestPath(Path workspaceRoot) {
    return workspaceRoot.resolve(MANIFEST_RELATIVE_PATH);
  }

  /** Invokes {@code ./gradlew writePreviewClasspathManifest} in {@code workspaceRoot}. */
  public static void refreshViaGradle(Path workspaceRoot) {
    requireNonNull(workspaceRoot);
    boolean isWindows = System.getProperty("os.name", "").toLowerCase().contains("win");
    Path wrapper = workspaceRoot.resolve(isWindows ? "gradlew.bat" : "gradlew");
    List<String> command =
        isWindows
            ? List.of(
                "cmd.exe",
                "/c",
                wrapper.toAbsolutePath().toString(),
                "writePreviewClasspathManifest",
                "--console=plain",
                "-q")
            : List.of(
                wrapper.toAbsolutePath().toString(),
                "writePreviewClasspathManifest",
                "--console=plain",
                "-q");
    try {
      Process proc =
          new ProcessBuilder(command)
              .directory(workspaceRoot.toFile())
              .redirectErrorStream(true)
              .start();
      String output = new String(proc.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
      int code = proc.waitFor();
      if (code != 0) {
        throw new IllegalStateException(
            "Gradle classpath manifest refresh failed (exit " + code + "): " + output);
      }
    } catch (IOException | InterruptedException e) {
      if (e instanceof InterruptedException) {
        Thread.currentThread().interrupt();
      }
      throw new IllegalStateException("Failed to run Gradle classpath manifest refresh", e);
    }
  }

  /**
   * Writes the classpath manifest atomically to {@code
   * .gradle/trailblazer-preview/classpath.manifest}.
   */
  public static ClasspathManifest writeManifest(
      Path workspaceRoot,
      List<String> classpathEntries,
      String jniLibraryPath,
      List<String> sourceRoots) {
    requireNonNull(workspaceRoot);
    requireNonNull(classpathEntries);
    requireNonNull(jniLibraryPath);
    requireNonNull(sourceRoots);
    String hash = computeBuildFilesHash(workspaceRoot);
    ClasspathManifest manifest =
        new ClasspathManifest(hash, classpathEntries, jniLibraryPath, sourceRoots);
    Path target = manifestPath(workspaceRoot);
    try {
      Files.createDirectories(target.getParent());
      MAPPER.writerWithDefaultPrettyPrinter().writeValue(target.toFile(), manifest);
      return manifest;
    } catch (IOException e) {
      throw new IllegalStateException("Failed to write classpath manifest to " + target, e);
    }
  }

  private ClasspathManifestManager() {}
}
