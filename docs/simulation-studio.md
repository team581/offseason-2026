# Standalone Simulation Studio cutover

Run `./gradlew offseason-bot:simulateJava` or WPILib: Simulate Robot Code in VS Code. If [Simulation Studio](https://github.com/simonstoryparker/simulation-studio/releases/latest) is installed, it opens automatically. Otherwise, the launcher suggests installing it and continues with the default WPILib simulation GUI. No Node or Rust is needed.

The Java bridge resolves `/Applications/Simulation Studio.app`, `~/Applications/Simulation Studio.app`, or `%LOCALAPPDATA%\Simulation Studio\simulation-studio.exe` at launch. `TEAM581_SIM_APP` overrides discovery, including for local app development. It must name a bundle or executable, not a command with arguments.

Use `-PwpilibSimGui=true` to explicitly select the WPILib GUI even when Simulation Studio is installed. VS Code external simulation metadata uses the detected installation when generated; regenerate it after installing or removing Simulation Studio.

## Standalone release

[Simulation Studio v0.1.1](https://github.com/simonstoryparker/simulation-studio/releases/tag/v0.1.1) is published. Native macOS and Windows [installation and updater acceptance](https://github.com/simonstoryparker/simulation-studio/actions/runs/38797334713) passed, including manual replacement, signed automatic updates, obsolete-resource removal, workspace preservation, failed downloads/signatures, and disabled restart. A real macOS 0.1.0 → 0.1.1 update through the public feed also preserved the saved layout and preferences.

Following Dashshund's distribution approach, macOS uses an ad-hoc-signed DMG and Windows uses an NSIS installer without an OS publisher certificate. First installation can require OS approval. Updater signatures are mandatory on both platforms; no login or Node/Rust installation is required. The embedded app is maintained in the public repository rather than this robot branch.

## Test ownership

Frontend, Rust, UI, input mapping, workspace migration, packaging and updater tests live in the public app repository. All original tests under `sim-app/` move there. The original UI requirements in the Java test directory also move there.

Java control bridge tests, HAL/NT fixture, installed-app discovery tests, and robot behavior tests stay here. See `shared/src/test/resources/simulation/control-v1.json` for the protocol vectors also tested by the app. Robot CI never builds or tests the desktop app.

Robot-side integration check: `./gradlew shared:test --tests 'com.team581.simulation.*Test' -PwpilibSimGui=true`.
