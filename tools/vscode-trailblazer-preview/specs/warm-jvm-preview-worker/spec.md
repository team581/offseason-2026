# Java preview worker

Keep a Java worker running so edits can refresh quickly. Aim for under 3 seconds on the first preview and under 1 second on a warm refresh.

- Start the worker in the background in trusted workspaces only. Stop it when the extension deactivates.
- Require a Java 21 JDK with a compiler. If the compiler is missing, explain the requirement and exit with an error.
- Run the robot's `PathBuilder` and closed-loop `Trailblazer` code for the selected robot and alliance.
- Start a single segment from the previous segment's end pose, or from the auto's start pose for the first segment. Run all segments in sequence for Whole Auto.
- Reuse the cached Gradle classpath and refresh it when build files or dependencies change.
- Compile edits and Git baseline sources in isolated directories without overwriting Gradle output or working-tree classes. Use a fresh child classloader for each run and close it on success or failure.
- Cancel superseded requests and discard their results.
