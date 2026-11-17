package com.team581.trailblazer3.preview;

import static com.google.common.base.Preconditions.checkState;
import static java.nio.charset.StandardCharsets.UTF_8;
import static java.util.Comparator.reverseOrder;
import static java.util.Objects.requireNonNull;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.preview.ClasspathManifestManager.ClasspathManifest;
import com.team581.trailblazer3.preview.PreviewRunner.PreviewRequest;
import com.team581.trailblazer3.preview.PreviewSchema.BuildIdentity;
import com.team581.trailblazer3.preview.PreviewSchema.DiagnosticEntry;
import com.team581.trailblazer3.preview.PreviewSchema.PreviewResult;
import com.team581.trailblazer3.preview.PreviewSchema.SegmentDescriptor;
import dev.doglog.DogLog;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintStream;
import java.io.StringWriter;
import java.net.URI;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Stream;
import javax.tools.Diagnostic;
import javax.tools.DiagnosticCollector;
import javax.tools.JavaCompiler;
import javax.tools.JavaFileObject;
import javax.tools.SimpleJavaFileObject;
import javax.tools.StandardJavaFileManager;
import javax.tools.ToolProvider;
import org.jspecify.annotations.Nullable;

/**
 * Long-lived Java 21 JDK stdio worker and headless CLI runner for Trailblazer 3 previews.
 *
 * <p>Incrementally compiles modified Java auto files into an isolated overlay directory, loads
 * classes through a disposable child {@link URLClassLoader} closed inside {@code
 * try-with-resources}, cancels superseded {@code requestId} runs, and executes the exact robot
 * {@code PathBuilder} and closed-loop {@code Trailblazer} code.
 */
public final class WarmPreviewWorker {
  /**
   * Disposable child classloader that loads freshly compiled classes from the isolated overlay
   * directory into memory before delegating to the parent classloader, preventing Windows file
   * locks and stale static class state.
   */
  public static final class OverlayClassLoader extends URLClassLoader {
    private final Path overlayDir;

    public OverlayClassLoader(Path overlayDir, ClassLoader parent) throws IOException {
      super(new URL[] {overlayDir.toUri().toURL()}, parent);
      this.overlayDir = requireNonNull(overlayDir);
    }

    @Override
    protected Class<?> loadClass(String name, boolean resolve) throws ClassNotFoundException {
      synchronized (getClassLoadingLock(name)) {
        Class<?> loaded = findLoadedClass(name);
        if (loaded == null
            && !name.startsWith("java.")
            && !name.startsWith("jdk.")
            && !name.startsWith("edu.wpi.")
            && !name.startsWith("com.team581.trailblazer3.")) {
          Path classFile = overlayDir.resolve(name.replace('.', '/') + ".class");
          if (Files.isRegularFile(classFile)) {
            try {
              byte[] bytes = Files.readAllBytes(classFile);
              loaded = defineClass(name, bytes, 0, bytes.length);
            } catch (IOException e) {
              throw new ClassNotFoundException("Failed to read overlay class " + name, e);
            }
          }
        }
        if (loaded == null) {
          loaded = super.loadClass(name, false);
        }
        if (resolve) {
          resolveClass(loaded);
        }
        return loaded;
      }
    }
  }

  /** Response envelope emitted over stdio JSON-RPC. */
  @JsonIgnoreProperties(ignoreUnknown = true)
  public record WorkerResponse(
      @Nullable String id,
      String type,
      boolean ok,
      boolean stale,
      @Nullable String errorCode,
      @Nullable String errorMessage,
      @Nullable List<DiagnosticEntry> diagnostics,
      @Nullable List<SegmentDescriptor> segments,
      @Nullable PreviewResult result) {}

  private static final class InMemoryJavaSource extends SimpleJavaFileObject {
    private final String code;

    InMemoryJavaSource(URI uri, String code) {
      super(uri, Kind.SOURCE);
      this.code = code;
    }

    @Override
    public CharSequence getCharContent(boolean ignoreEncodingErrors) {
      return code;
    }
  }

  private static final class Pattern {
    static String quote(String s) {
      return java.util.regex.Pattern.quote(s);
    }
  }

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private static final AtomicLong OVERLAY_COUNTER = new AtomicLong(1);

  /**
   * Entrypoint supporting both persistent stdio JSON-RPC mode ({@code --stdio}) and headless
   * single-shot CLI mode ({@code --auto <Name>}).
   */
  public static void main(String[] args) throws Exception {
    PrintStream protocolOut = System.out;
    System.setOut(System.err);

    JavaCompiler compiler = resolveSystemCompiler();
    if (compiler == null) {
      String errJson =
          MAPPER.writeValueAsString(
              ImmutableMap.of(
                  "type",
                  "error",
                  "code",
                  "MISSING_JAVA_COMPILER",
                  "message",
                  "Java 21 JDK with javac is required (ToolProvider.getSystemJavaCompiler() returned null)"));
      protocolOut.println(errJson);
      protocolOut.flush();
      System.err.println(errJson);
      System.exit(2);
      return;
    }

    Path workspace = Path.of(System.getProperty("user.dir")).toAbsolutePath().normalize();
    boolean stdioMode = false;
    boolean writeManifestOnly = false;
    String jniDirArg = System.getProperty("java.library.path", "");
    String autoArg = null;
    String scopeArg = "WHOLE_AUTO";
    String segmentArg = null;
    String allianceArg = "Blue";
    String robotArg = "581";
    long seedArg = 581L;

    for (int i = 0; i < args.length; i++) {
      String arg = args[i];
      switch (arg) {
        case "--stdio" -> stdioMode = true;
        case "--write-manifest" -> writeManifestOnly = true;
        case "--jni-dir" -> {
          if (i + 1 < args.length) {
            jniDirArg = args[++i];
          }
        }
        case "--workspace" -> {
          if (i + 1 < args.length) {
            workspace = Path.of(args[++i]).toAbsolutePath().normalize();
          }
        }
        case "--auto" -> {
          if (i + 1 < args.length) {
            autoArg = args[++i];
          }
        }
        case "--scope" -> {
          if (i + 1 < args.length) {
            scopeArg = args[++i];
          }
        }
        case "--segment" -> {
          if (i + 1 < args.length) {
            segmentArg = args[++i];
          }
        }
        case "--alliance" -> {
          if (i + 1 < args.length) {
            allianceArg = args[++i];
          }
        }
        case "--robot" -> {
          if (i + 1 < args.length) {
            robotArg = args[++i];
          }
        }
        case "--seed" -> {
          if (i + 1 < args.length) {
            seedArg = Long.parseLong(args[++i]);
          }
        }
        default -> {
          if (arg.startsWith("--auto=")) {
            autoArg = arg.substring("--auto=".length());
          } else if (arg.startsWith("--alliance=")) {
            allianceArg = arg.substring("--alliance=".length());
          } else if (arg.startsWith("--robot=")) {
            robotArg = arg.substring("--robot=".length());
          } else if (arg.startsWith("--scope=")) {
            scopeArg = arg.substring("--scope=".length());
          } else if (arg.startsWith("--segment=")) {
            segmentArg = arg.substring("--segment=".length());
          }
        }
      }
    }

    WarmPreviewWorker worker = new WarmPreviewWorker(workspace);

    if (writeManifestOnly) {
      String sysCp = System.getProperty("java.class.path", "");
      List<String> entries = List.of(sysCp.split(Pattern.quote(File.pathSeparator)));
      List<String> sourceRoots = defaultSourceRoots(workspace);
      ClasspathManifestManager.writeManifest(workspace, entries, jniDirArg, sourceRoots);
      return;
    }

    if (!stdioMode && autoArg != null) {
      prewarmRuntime();
      var paramsNode = MAPPER.createObjectNode();
      paramsNode.put("autoClass", autoArg);
      paramsNode.put("scope", segmentArg != null ? "SEGMENT" : scopeArg);
      if (segmentArg != null) {
        paramsNode.put("segmentId", segmentArg);
      }
      paramsNode.put("alliance", allianceArg);
      paramsNode.put("robotConfigId", robotArg);
      paramsNode.put("seed", seedArg);
      WorkerResponse resp = worker.executeRequest("cli-1", paramsNode, () -> {});
      if (!resp.ok() || resp.result() == null) {
        System.err.println("Preview failed: " + resp.errorCode() + " - " + resp.errorMessage());
        System.exit(1);
        return;
      }
      protocolOut.println(PreviewSchema.toJson(resp.result()));
      protocolOut.flush();
      return;
    }

    // Stdio JSON-RPC worker mode
    Thread prewarmThread =
        new Thread(
            () -> {
              try {
                prewarmRuntime();
                worker.ensureClasspathManifest();
              } catch (Throwable ignored) {
                // Handled per request
              }
            },
            "trailblazer-preview-prewarm");
    prewarmThread.setDaemon(true);
    prewarmThread.start();

    synchronized (protocolOut) {
      protocolOut.println(
          MAPPER.writeValueAsString(
              ImmutableMap.of(
                  "type",
                  "ready",
                  "javaVersion",
                  Runtime.version().toString(),
                  "compilerAvailable",
                  true)));
      protocolOut.flush();
    }

    ExecutorService executor =
        Executors.newSingleThreadExecutor(r -> {
          Thread t = new Thread(r, "trailblazer-preview-runner");
          t.setDaemon(true);
          return t;
        });
    Future<?> activeFuture = null;

    try (BufferedReader reader = new BufferedReader(new InputStreamReader(System.in, UTF_8))) {
      String line;
      while ((line = reader.readLine()) != null) {
        String trimmed = line.trim();
        if (trimmed.isEmpty()) {
          continue;
        }
        JsonNode msg;
        try {
          msg = MAPPER.readTree(trimmed);
        } catch (IOException e) {
          continue;
        }
        String id = textOrDefault(msg, "id", "req-" + System.nanoTime());
        String method = textOrDefault(msg, "method", "preview");
        if ("shutdown".equals(method) || "exit".equals(method)) {
          synchronized (protocolOut) {
            protocolOut.println(
                MAPPER.writeValueAsString(
                    ImmutableMap.of("id", id, "type", "shutdown", "ok", true)));
            protocolOut.flush();
          }
          break;
        }
        if ("ping".equals(method)) {
          synchronized (protocolOut) {
            protocolOut.println(
                MAPPER.writeValueAsString(ImmutableMap.of("id", id, "type", "pong", "ok", true)));
            protocolOut.flush();
          }
          continue;
        }

        JsonNode params = msg.has("params") ? msg.get("params") : msg;
        worker.setLatestRequestId(id);
        if (activeFuture != null && !activeFuture.isDone()) {
          activeFuture.cancel(true);
        }
        final String reqId = id;
        activeFuture =
            executor.submit(() -> {
              Runnable checkCancelled =
                  () -> {
                    if (!reqId.equals(worker.latestRequestId.get())
                        || Thread.currentThread().isInterrupted()) {
                      throw new CancellationException("Superseded request " + reqId);
                    }
                  };
              try {
                WorkerResponse response = worker.executeRequest(reqId, params, checkCancelled);
                checkCancelled.run();
                String json = MAPPER.writeValueAsString(response);
                synchronized (protocolOut) {
                  if (reqId.equals(worker.latestRequestId.get())) {
                    protocolOut.println(json);
                    protocolOut.flush();
                  }
                }
              } catch (CancellationException ignored) {
                // Superseded by newer request; discard output per spec
              } catch (Throwable t) {
                try {
                  WorkerResponse errResp =
                      new WorkerResponse(
                          reqId,
                          "result",
                          false,
                          true,
                          "INTERNAL_ERROR",
                          t.getMessage(),
                          ImmutableList.of(),
                          null,
                          null);
                  synchronized (protocolOut) {
                    if (reqId.equals(worker.latestRequestId.get())) {
                      protocolOut.println(MAPPER.writeValueAsString(errResp));
                      protocolOut.flush();
                    }
                  }
                } catch (IOException ignored) {
                  // ignore
                }
              }
            });
      }
    } finally {
      executor.shutdownNow();
    }
  }

  /** Pre-warms WPILib JNI and core Trailblazer math classes in the parent classloader. */
  public static void prewarmRuntime() {
    try {
      HAL.initialize(500, 0);
    } catch (Throwable ignored) {
      // Continue if JNI path is loaded via manifest or already initialized
    }
    try {
      DogLog.setEnabled(false);
    } catch (Throwable ignored) {
      // Ignore if DogLog is not initialized
    }
    var profile = PreviewRunner.robotProfile("581");
    var clearance = PreviewRunner.buildClearance(profile, Alliance.Blue);
    var warmPath =
        PathBuilder.build(
            new Translation2d(1.0, 1.0),
            ImmutableList.of(Goal.stop(new Pose2d(2.0, 1.0, Rotation2d.kZero))),
            profile.calibration().maxBrakingDeceleration(),
            clearance);
    warmPath.sample(0.5);
  }

  /** Verifies that a system Java compiler is available (unless simulated missing for tests). */
  public static @Nullable JavaCompiler resolveSystemCompiler() {
    if (Boolean.getBoolean("trailblazer.preview.simulateMissingCompiler")) {
      return null;
    }
    return ToolProvider.getSystemJavaCompiler();
  }

  private static List<String> defaultSourceRoots(Path workspaceRoot) {
    List<String> roots = new ArrayList<>();
    for (String sub : ImmutableList.of("comp-bot", "shared", "offseason-bot", "new-member-bot")) {
      Path src = workspaceRoot.resolve(sub).resolve("src/main/java");
      if (Files.isDirectory(src)) {
        roots.add(src.toAbsolutePath().toString());
      }
    }
    return roots;
  }

  private static void deleteDirectoryQuietly(Path dir) {
    if (dir == null || !Files.exists(dir)) {
      return;
    }
    try (Stream<Path> walk = Files.walk(dir)) {
      walk.sorted(reverseOrder())
          .forEach(p -> {
            try {
              Files.deleteIfExists(p);
            } catch (IOException ignored) {
              // ignore
            }
          });
    } catch (IOException ignored) {
      // ignore
    }
  }

  private static String sha256Hex(String input) {
    try {
      MessageDigest md = MessageDigest.getInstance("SHA-256");
      return HexFormat.of().formatHex(md.digest(input.getBytes(UTF_8)));
    } catch (NoSuchAlgorithmException e) {
      return Integer.toHexString(input.hashCode());
    }
  }

  private static String textOrDefault(JsonNode node, String field, String fallback) {
    String v = textOrNull(node, field);
    return v != null && !v.isBlank() ? v : fallback;
  }

  private static @Nullable String textOrNull(JsonNode node, String field) {
    if (node == null || !node.has(field) || node.get(field).isNull()) {
      return null;
    }
    return node.get(field).asText();
  }

  private final Path workspaceRoot;

  private final JavaCompiler compiler;

  private final AtomicReference<String> latestRequestId = new AtomicReference<>("");

  private volatile @Nullable ClasspathManifest cachedManifest;

  private volatile @Nullable URLClassLoader baseClasspathLoader;

  private volatile @Nullable String baseClasspathHash;

  public WarmPreviewWorker(Path workspaceRoot) {
    this.workspaceRoot = requireNonNull(workspaceRoot).toAbsolutePath().normalize();
    this.compiler = resolveSystemCompiler();
    checkState(
        this.compiler != null,
        "Java 21 JDK with javac is required (ToolProvider.getSystemJavaCompiler() returned null)");
  }

  /** Executes a preview request (compiling overlay if source is provided) and returns response. */
  public WorkerResponse executeRequest(
      @Nullable String requestId, JsonNode params, Runnable checkCancelled) {
    requireNonNull(params);
    requireNonNull(checkCancelled);
    checkCancelled.run();

    String sourceFileParam = textOrNull(params, "sourceFile");
    String sourceContentParam = textOrNull(params, "sourceContent");
    String autoClassParam = textOrNull(params, "autoClass");
    String scopeParam = textOrDefault(params, "scope", "CURSOR");
    String segmentIdParam = textOrNull(params, "segmentId");
    int cursorLine = params.has("cursorLine") ? params.get("cursorLine").asInt(0) : 0;
    String previousSegmentId = textOrNull(params, "previousSegmentId");
    String allianceParam = textOrDefault(params, "alliance", "Blue");
    String robotParam = textOrDefault(params, "robotConfigId", "581");
    long seed = params.has("seed") ? params.get("seed").asLong(581L) : 581L;

    Path sourcePath = null;
    String sourceText = sourceContentParam != null ? sourceContentParam : "";
    if (sourceFileParam != null && !sourceFileParam.isBlank()) {
      sourcePath = Path.of(sourceFileParam);
      if (!sourcePath.isAbsolute()) {
        sourcePath = workspaceRoot.resolve(sourcePath).normalize();
      }
      if (sourceText.isEmpty() && Files.isRegularFile(sourcePath)) {
        try {
          sourceText = Files.readString(sourcePath);
        } catch (IOException e) {
          return new WorkerResponse(
              requestId,
              "result",
              false,
              true,
              "SOURCE_READ_ERROR",
              "Failed to read source file: " + e.getMessage(),
              ImmutableList.of(
                  new DiagnosticEntry(
                      "ERROR",
                      "SOURCE_READ_ERROR",
                      "Failed to read " + sourcePath.getFileName(),
                      null,
                      null,
                      null,
                      null)),
              null,
              null);
        }
      }
    }

    String resolvedClassName = autoClassParam;
    if ((resolvedClassName == null || resolvedClassName.isBlank()) && !sourceText.isEmpty()) {
      resolvedClassName = SourceSpanAnalyzer.parseClassName(sourceText).orElse(null);
    }
    if (resolvedClassName != null && !resolvedClassName.contains(".")) {
      resolvedClassName = "frc.robot.autos." + resolvedClassName;
    }
    if (resolvedClassName == null || resolvedClassName.isBlank()) {
      return new WorkerResponse(
          requestId,
          "result",
          false,
          true,
          "MISSING_AUTO_CLASS",
          "Could not determine auto class name",
          ImmutableList.of(
              new DiagnosticEntry(
                  "ERROR", "MISSING_AUTO_CLASS", "MISSING_AUTO_CLASS", null, null, null, null)),
          null,
          null);
    }

    if (sourceText.isEmpty() && sourcePath == null) {
      sourcePath = findSourceFileForClass(resolvedClassName).orElse(null);
      if (sourcePath != null && Files.isRegularFile(sourcePath)) {
        try {
          sourceText = Files.readString(sourcePath);
        } catch (IOException ignored) {
          // Proceed without source text
        }
      }
    }

    checkCancelled.run();
    ClasspathManifest manifest = ensureClasspathManifest();
    ClassLoader parentLoader = ensureBaseClassLoader(manifest);

    Path overlayBaseDir = workspaceRoot.resolve(".gradle/trailblazer-preview/overlay");
    Path overlayDir = overlayBaseDir.resolve("run-" + OVERLAY_COUNTER.getAndIncrement());
    double compileMs = 0.0;

    try {
      Files.createDirectories(overlayDir);
      if (!sourceText.isEmpty()) {
        long compileStartNs = System.nanoTime();
        List<DiagnosticEntry> compileErrors =
            compileToOverlay(resolvedClassName, sourcePath, sourceText, manifest, overlayDir);
        compileMs = (System.nanoTime() - compileStartNs) / 1_000_000.0;
        checkCancelled.run();
        if (!compileErrors.isEmpty()) {
          List<Map.Entry<String, String>> rawKnown = new ArrayList<>();
          for (var raw : SourceSpanAnalyzer.parseRawSegments(sourceText)) {
            String id = raw.literalSegmentId() != null ? raw.literalSegmentId() : raw.fieldName();
            rawKnown.add(Map.entry(raw.fieldName(), id));
          }
          List<SegmentDescriptor> fallbackSegments =
              SourceSpanAnalyzer.analyze(sourceText, rawKnown);
          return new WorkerResponse(
              requestId,
              "result",
              false,
              true,
              "COMPILE_ERROR",
              compileErrors.get(0).message(),
              compileErrors,
              fallbackSegments,
              null);
        }
      }

      checkCancelled.run();
      try (OverlayClassLoader childLoader = new OverlayClassLoader(overlayDir, parentLoader)) {
        Class<?> autoClass = Class.forName(resolvedClassName, false, childLoader);
        String sourceHash = sha256Hex(sourceText);
        BuildIdentity identity =
            new BuildIdentity(sourceHash, manifest.buildFilesHash(), System.currentTimeMillis());
        PreviewRequest req =
            new PreviewRequest(
                resolvedClassName,
                scopeParam,
                segmentIdParam,
                cursorLine,
                previousSegmentId,
                allianceParam,
                robotParam,
                seed,
                sourceText,
                compileMs,
                identity);
        PreviewResult previewResult = PreviewRunner.run(autoClass, req, checkCancelled);
        checkCancelled.run();
        return new WorkerResponse(
            requestId,
            "result",
            true,
            false,
            null,
            null,
            previewResult.diagnostics(),
            previewResult.segments(),
            previewResult);
      }
    } catch (CancellationException cancelled) {
      throw cancelled;
    } catch (Throwable ex) {
      String msg = ex.getMessage() != null ? ex.getMessage() : ex.getClass().getSimpleName();
      return new WorkerResponse(
          requestId,
          "result",
          false,
          true,
          "RUNTIME_ERROR",
          msg,
          ImmutableList.of(
              new DiagnosticEntry("ERROR", "RUNTIME_ERROR", msg, null, null, null, null)),
          null,
          null);
    } finally {
      deleteDirectoryQuietly(overlayDir);
    }
  }

  /** Sets the latest active request ID, cancelling any earlier superseded request IDs. */
  public void setLatestRequestId(String requestId) {
    latestRequestId.set(requireNonNull(requestId));
  }

  private ImmutableList<DiagnosticEntry> compileToOverlay(
      String className,
      @Nullable Path sourcePath,
      String sourceText,
      ClasspathManifest manifest,
      Path overlayDir)
      throws IOException {
    DiagnosticCollector<JavaFileObject> collector = new DiagnosticCollector<>();
    try (StandardJavaFileManager fileManager =
        compiler.getStandardFileManager(collector, Locale.ROOT, UTF_8)) {
      URI uri =
          sourcePath != null
              ? sourcePath.toUri()
              : URI.create("string:///" + className.replace('.', '/') + ".java");
      JavaFileObject fileObject = new InMemoryJavaSource(uri, sourceText);

      List<String> options = new ArrayList<>();
      options.add("--release");
      options.add("17");
      options.add("-XDstringConcat=inline");
      options.add("-g:lines,source");
      options.add("-proc:none");
      options.add("-Xprefer:newer");
      options.add("-implicit:none");
      String cp = manifest.toClasspathString();
      if (!cp.isEmpty()) {
        options.add("-classpath");
        options.add(cp);
      }
      String sp = manifest.toSourcepathString();
      if (!sp.isEmpty()) {
        options.add("-sourcepath");
        options.add(sp);
      }
      options.add("-d");
      options.add(overlayDir.toAbsolutePath().toString());

      StringWriter compilerOut = new StringWriter();
      JavaCompiler.CompilationTask task =
          compiler.getTask(
              compilerOut, fileManager, collector, options, null, ImmutableList.of(fileObject));
      boolean success = Boolean.TRUE.equals(task.call());
      if (success) {
        return ImmutableList.of();
      }

      List<DiagnosticEntry> errors = new ArrayList<>();
      for (Diagnostic<? extends JavaFileObject> d : collector.getDiagnostics()) {
        if (d.getKind() == Diagnostic.Kind.ERROR) {
          int line = d.getLineNumber() > 0 ? (int) d.getLineNumber() : 1;
          String rawMsg = d.getMessage(Locale.ROOT);
          String firstLine = rawMsg != null ? rawMsg.split("\\R", 2)[0].trim() : "Compile error";
          errors.add(
              new DiagnosticEntry(
                  "ERROR", "COMPILE_ERROR", "L" + line + ": " + firstLine, null, null, null, line));
        }
      }
      if (errors.isEmpty()) {
        errors.add(
            new DiagnosticEntry(
                "ERROR",
                "COMPILE_ERROR",
                "Compilation failed: " + compilerOut.toString().trim(),
                null,
                null,
                null,
                1));
      }
      return ImmutableList.copyOf(errors);
    }
  }

  private synchronized ClassLoader ensureBaseClassLoader(ClasspathManifest manifest) {
    if (baseClasspathLoader != null && manifest.buildFilesHash().equals(baseClasspathHash)) {
      return baseClasspathLoader;
    }
    try {
      List<URL> urls = new ArrayList<>();
      for (String entry : manifest.classpathEntries()) {
        Path p = Path.of(entry);
        if (Files.exists(p)) {
          urls.add(p.toUri().toURL());
        }
      }
      baseClasspathLoader =
          new URLClassLoader(urls.toArray(URL[]::new), WarmPreviewWorker.class.getClassLoader());
      baseClasspathHash = manifest.buildFilesHash();
      return baseClasspathLoader;
    } catch (IOException e) {
      return WarmPreviewWorker.class.getClassLoader();
    }
  }

  private synchronized ClasspathManifest ensureClasspathManifest() {
    Optional<ClasspathManifest> valid = ClasspathManifestManager.loadIfValid(workspaceRoot);
    if (valid.isPresent()) {
      cachedManifest = valid.orElseThrow();
      return cachedManifest;
    }
    // Check if current process classpath already has the project classes (e.g. launched via
    // Gradle/tests)
    String sysCp = System.getProperty("java.class.path", "");
    if (!sysCp.isBlank() && Files.exists(workspaceRoot.resolve("build.gradle"))) {
      List<String> entries = List.of(sysCp.split(Pattern.quote(File.pathSeparator)));
      List<String> sourceRoots = defaultSourceRoots(workspaceRoot);
      String jniDir = System.getProperty("java.library.path", "");
      try {
        // If build.gradle exists and we had a previous manifest whose hash changed, refresh via
        // Gradle
        if (cachedManifest != null && Files.isRegularFile(workspaceRoot.resolve("gradlew"))) {
          cachedManifest = ClasspathManifestManager.loadOrRefresh(workspaceRoot);
          return cachedManifest;
        }
      } catch (RuntimeException ignored) {
        // Fallback to writing current process classpath
      }
      cachedManifest =
          ClasspathManifestManager.writeManifest(workspaceRoot, entries, jniDir, sourceRoots);
      return cachedManifest;
    }
    cachedManifest = ClasspathManifestManager.loadOrRefresh(workspaceRoot);
    return cachedManifest;
  }

  private Optional<Path> findSourceFileForClass(String className) {
    String relFile = className.replace('.', '/') + ".java";
    for (String root : defaultSourceRoots(workspaceRoot)) {
      Path candidate = Path.of(root).resolve(relFile);
      if (Files.isRegularFile(candidate)) {
        return Optional.of(candidate);
      }
    }
    return Optional.empty();
  }
}
