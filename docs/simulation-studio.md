# Standalone Simulation Studio cutover

Install [Simulation Studio](https://github.com/simonstoryparker/simulation-studio/releases/latest), then run `./gradlew offseason-bot:simulateJava` or WPILib: Simulate Robot Code in VS Code. No Node or Rust is needed.

The Java bridge resolves `/Applications/Simulation Studio.app`, `~/Applications/Simulation Studio.app`, or `%LOCALAPPDATA%\Simulation Studio\simulation-studio.exe` at launch. `TEAM581_SIM_APP` overrides discovery, including for local app development. It must name a bundle or executable, not a command with arguments.

Use `-PwpilibSimGui=true` to restore the WPILib GUI. Regenerate external simulation metadata with that property before using the fallback in VS Code.

## Release gate

Do not merge this extraction branch or remove the embedded app from `simon-simulation-desktop-app` until the public standalone release is signed, published, and verified on clean macOS and Windows machines. The source branch remains intact while this candidate is reviewed.

## Test ownership

Frontend, Rust, UI, input mapping, workspace migration, packaging and updater tests live in the public app repository. All original tests under `sim-app/` move there. The original UI requirements in the Java test directory also move there.

Java control bridge tests, HAL/NT fixture, installed-app discovery tests, and robot behavior tests stay here. See `shared/src/test/resources/simulation/control-v1.json` for the protocol vectors also tested by the app. Robot CI never builds or tests the desktop app.

Robot-side integration check: `./gradlew shared:test --tests 'com.team581.simulation.*Test' -PwpilibSimGui=true`.
