#!/usr/bin/env python3
"""Compare branch and working-tree autos with the VS Code preview worker."""

import argparse
import hashlib
import io
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import tarfile
import time


ROOT = Path(__file__).resolve().parents[1]
WORKER = "com.team581.trailblazer3.preview.WarmPreviewWorker"


def run(args, cwd=ROOT, **kwargs):
    return subprocess.run(args, cwd=cwd, check=True, **kwargs)


def build(root):
    run([str(root / "gradlew"), ":comp-bot:writePreviewClasspathManifest",
         "--console=plain", "-q"], cwd=root)
    return json.loads((root / ".gradle/trailblazer-preview/classpath.manifest").read_text())


def baseline(ref):
    revision = run(["git", "rev-parse", "--verify", f"{ref}^{{commit}}"],
                   capture_output=True, text=True).stdout.strip()
    checkout = ROOT / ".gradle/trailblazer-compare" / revision
    if not checkout.exists():
        checkout.mkdir(parents=True)
        try:
            archive = run(["git", "archive", revision], capture_output=True).stdout
            with tarfile.open(fileobj=io.BytesIO(archive)) as files:
                files.extractall(checkout, filter="data")
        except BaseException:
            shutil.rmtree(checkout)
            raise
    return revision, checkout


def behavior(result):
    def clean(value):
        if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items() if k not in {"sourceLine", "sourceEndLine", "line"}}
        if isinstance(value, list):
            return [clean(v) for v in value]
        return value

    return clean({k: result[k] for k in (
        "totalDurationSeconds", "startPose", "waypoints", "mathPath", "keepouts",
        "trajectory", "passedMarkers", "headingArrivals", "dominantLimitingFactor",
        "minSlowdownRatio", "diagnostics")})


def previews(root, manifest, cases):
    java_home = os.environ.get("JAVA_HOME")
    java = str(Path(java_home) / "bin/java") if java_home else shutil.which("java")
    wpilib_java = Path.home() / "wpilib/2026/jdk/bin/java"
    if not java_home and wpilib_java.exists():
        java = str(wpilib_java)
    native = manifest["jniLibraryPath"]
    env = dict(os.environ, LD_LIBRARY_PATH=native, DYLD_LIBRARY_PATH=native,
               DYLD_FALLBACK_LIBRARY_PATH=native)
    log = root / ".gradle/trailblazer-preview/compare-worker.log"
    results = {}
    with log.open("w") as errors:
        process = subprocess.Popen(
            [java, f"-Djava.library.path={native}", "-cp",
             os.pathsep.join(manifest["classpathEntries"]), WORKER,
             "--stdio", "--workspace", str(root)],
            cwd=root, env=env, text=True, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=errors)
        try:
            def receive():
                while True:
                    line = process.stdout.readline()
                    if not line:
                        raise RuntimeError(f"Preview worker stopped; see {log}")
                    try:
                        return json.loads(line)
                    except json.JSONDecodeError:
                        errors.write(line)

            ready = receive()
            if ready.get("type") != "ready":
                raise RuntimeError(ready)
            for auto, alliance, robot in cases:
                request = {"id": "compare", "method": "preview", "params": {
                    "autoClass": auto, "scope": "WHOLE_AUTO", "alliance": alliance,
                    "robotConfigId": robot, "seed": 581}}
                process.stdin.write(json.dumps(request) + "\n")
                process.stdin.flush()
                response = receive()
                if not response.get("ok"):
                    raise RuntimeError(f"{auto}/{alliance}/{robot}: {response.get('errorMessage')}")
                result = response["result"]
                failures = [d for d in result["diagnostics"] if d["severity"] == "ERROR"]
                if failures:
                    raise RuntimeError(f"{auto}/{alliance}/{robot}: {failures}")
                results[(auto, alliance, robot)] = behavior(result)
        finally:
            process.terminate()
            process.wait()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", default="HEAD", help="Branch or commit (default: HEAD)")
    parser.add_argument("--auto", action="append", help="Auto class; repeat for multiple autos")
    parser.add_argument("--robot", action="append", choices=["581", "9991", "9981"])
    parser.add_argument("--output", type=Path, help="Save durations and behavior hashes as JSON")
    args = parser.parse_args()
    autos = args.auto or sorted(
        p.stem for p in (ROOT / "comp-bot/src/main/java/frc/robot/autos").glob("*.java")
        if re.search(r"\bextends\s+TrailblazerAuto<", p.read_text()))
    cases = [(a, side, robot) for a in autos for side in ("Blue", "Red")
             for robot in (args.robot or ["581"])]
    started = time.monotonic()
    revision, checkout = baseline(args.ref)
    cache = checkout / ".gradle/trailblazer-comparisons-v1"
    cache.mkdir(parents=True, exist_ok=True)
    paths = {case: cache / (hashlib.sha256(json.dumps(case).encode()).hexdigest() + ".json")
             for case in cases}
    old = {case: json.loads(path.read_text()) for case, path in paths.items() if path.exists()}
    missing = [case for case in cases if case not in old]
    if missing:
        old.update(previews(checkout, build(checkout), missing))
        for case in missing:
            paths[case].write_text(json.dumps(old[case]))
    new = previews(ROOT, build(ROOT), cases)
    rows = []
    print(f"\nBaseline: {revision[:12]} ({args.ref})\n")
    print("| Auto | Alliance | Robot | Branch (s) | Working (s) | Delta (s) | Output |")
    print("|---|---|---|---:|---:|---:|---|")
    for auto, alliance, robot in cases:
        key = (auto, alliance, robot)
        before, after = old[key]["totalDurationSeconds"], new[key]["totalDurationSeconds"]
        same = old[key] == new[key]
        hashes = [hashlib.sha256(json.dumps(r, sort_keys=True).encode()).hexdigest()
                  for r in (old[key], new[key])]
        rows.append(dict(auto=auto, alliance=alliance, robot=robot, branchSeconds=before,
                         workingSeconds=after, deltaSeconds=after - before,
                         outputMatches=same, branchHash=hashes[0], workingHash=hashes[1]))
        print(f"| {auto} | {alliance} | {robot} | {before:.3f} | {after:.3f} | "
              f"{after - before:+.3f} | {'same' if same else 'CHANGED'} |")
    print(f"\nElapsed: {time.monotonic() - started:.1f}s. Times cover driving segments, not shooting waits.")
    if args.output:
        args.output.write_text(json.dumps(dict(revision=revision, comparisons=rows), indent=2) + "\n")
    return 0 if all(r["outputMatches"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
