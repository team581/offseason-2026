# Trailblazer 3 Auto Preview

Split-screen VS Code extension for previewing Team 581 Trailblazer 3 autonomous routines and path segments.

## Install

- Install `trailblazer-preview-0.3.1.vsix` with VS Code's **Extensions: Install from VSIX...** command, then reload the window.
- The first preview builds saved Java dependencies. Waypoint edits reuse the warm worker; helper changes trigger a rebuild.

## Specs

- [VS Code preview](specs/vscode-preview-viewer/spec.md)
- [Java preview worker](specs/warm-jvm-preview-worker/spec.md)
- [Preview result data](specs/preview-result-schema/spec.md)
