# Developer tooling

- Author Java segments with `Trailblazer.segment(id)` and the existing fluent builder.
- Use `CompAutoField.fromRed` or `fromBlue` for any auto; both produce blue-frame poses.
- `AutoSegment.forAlliance` converts the segment before activation.
- Robot and desktop code share the field keepouts and calibration defaults.

## Preview

- `WarmPreviewWorker` compiles an edited auto into an overlay and runs `PreviewRunner`.
- The worker stays alive between requests and cancels superseded requests.
- `PreviewRunner` uses the same path builder and follower as the robot.
- The preview plant models response lag; it does not run mechanisms or the bump tracker.
- Source spans connect preview points to Java lines and resolve cursor-selected segments.
- The payload includes path geometry, waypoints, trajectory, markers, heading arrivals, and diagnostics.

## Commands

```sh
./gradlew :comp-bot:previewAuto -Pauto=RightTrenchAuto -Palliance=Red
python3 tools/compare_tb3.py --auto RightTrenchAuto --auto LeftTrenchAuto
```

- The comparison tool builds the working tree and an isolated copy of the chosen Git revision.
- Cached branch builds avoid rebuilding unchanged code.
- A source-only overlay is insufficient when shared math or config has changed.
- See the [authoring guide](../../../../../../../../docs/writing-autos-tb3.md#compare-changes) for options.
