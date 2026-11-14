# Trailblazer 3 VS Code Preview App

**Status:** Deferred. Begin implementation only when explicitly requested.

The app is a separate project that consumes the Java scenarios, simulation engine, and versioned results documented in the [headless verification spec](../05-developer-tooling.md#4-headless-verification). The [Trailblazer 3 roadmap](../08-implementation-roadmap-and-migration.md) can be completed independently.

## Warm-JVM Stdio Worker

Launch under the full **Java 21 JDK** toolchain (`ToolProvider.getSystemJavaCompiler()` returns `null` under a JRE). Reads a cached Gradle classpath manifest on startup, compiles only modified Java plan/scenario files incrementally into an **isolated overlay output directory**, and loads scenario classes through a fresh disposable child `URLClassLoader` (with the overlay directory placed ahead of the cached Gradle classpath) closed inside `try-with-resources` so Windows JAR/class locks and stale class state cannot accumulate. If `build.gradle` or dependencies change, the worker refreshes the classpath manifest automatically.

## Viewer

1. **Selection:** Choose scenario, robot configuration, alliance, and seed.
2. **Refresh:** Invoke the warm stdio preview worker on command or debounced save (falling back to Gradle when build scripts change); no separate Java parser or planner in TypeScript.
3. **View:** Display goals, fillets, keepouts (`K_hard` and `K_margin`), swept footprint, motion/heading playback, velocity graphs, markers, and limiting factors.
4. **Diagnostics:** Show compile errors and failed assertions. Cancel superseded runs, discard stale results, and label the last successful preview stale after a failed build.
5. **Integration:** Respect workspace trust, clean up worker/child processes and watchers on shutdown, and test both the warm worker and headless verification harness against the same versioned output schema.

## Acceptance

The app is ready when a student can edit a Java plan, see it run in VS Code in sub-second warm refreshes, locate a failed constraint, and reproduce the same result with the temporary headless harness. Test compile errors, cancellation, stale results, Windows file-lock safety, and robot/alliance selection; measure cold/warm refresh times.
