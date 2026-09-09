# Robot loop profiling

The loop analyzer reads one or more WPILOG files, removes the first two seconds after startup and
each Driver Station mode transition, and emits machine-readable JSON plus a Markdown report.

```sh
cd shared/src/main/python/profiling
uv run python analyze_loop_timing.py \
  ../../../../comp-bot/logs/FRC_20260907_194151.wpilog \
  --mode auto --output auto.json --markdown auto.md

uv run python analyze_loop_timing.py candidate.wpilog \
  --mode auto --baseline auto.json --output candidate.json --markdown candidate.md
```

The analyzer refuses to combine different build SHAs unless `--allow-mixed-builds` is supplied.
Raw WPILOG files stay ignored and must not be committed.

The comp-bot desktop benchmark runs a fixed 20-second disabled warmup followed by a 15-second
autonomous or 30-second teleop workload. Each aggregate task uses five fresh JVMs and records JSON
and JFR under `comp-bot/build/reports/loop-profile`.

```sh
./gradlew :comp-bot:loopBenchmarkBaseline
./gradlew :comp-bot:loopBenchmarkReport \
  -PloopBenchmarkBaseline=comp-bot/build/reports/loop-profile/baseline.json
```

Open a `.jfr` file in JDK Mission Control, IntelliJ, or VisualVM's JFR viewer to rank main-thread
exclusive CPU and allocations. These laptop results are for relative comparisons; they do not
prove absolute roboRIO loop timing. Native CAN refresh is instrumented on the robot but excluded
from the laptop simulation because vendor simulation cannot reproduce real CAN/CAN-FD timing.
