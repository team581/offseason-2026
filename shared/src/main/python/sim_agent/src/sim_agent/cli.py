"""sim-agent CLI: run a scenario against the WPILib simulator.

Usage:
    uv run --package sim-agent sim-agent drive_and_score            # built-in scenario
    uv run --package sim-agent sim-agent /path/to/my_scenario.py    # scenario file
    uv run --package sim-agent sim-agent drive_and_score --attach   # attach to running sim

A scenario is a Python module with a ``run(session: SimSession) -> None`` function.
AssertionErrors (and timeouts from session.wait_*) fail the run with exit code 1.
"""

import argparse
import importlib
import importlib.util
import sys
import time
import traceback
from pathlib import Path
from types import ModuleType

from sim_agent.session import SimSession


def _load_scenario(name_or_path: str) -> ModuleType:
    path = Path(name_or_path)
    if path.suffix == ".py" and path.exists():
        spec = importlib.util.spec_from_file_location("sim_agent_user_scenario", path)
        if spec is None or spec.loader is None:
            raise ValueError(f"Cannot load scenario file: {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    return importlib.import_module(f"sim_agent.scenarios.{name_or_path}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="sim-agent", description=__doc__)
    parser.add_argument(
        "scenario", help="Built-in scenario name or path to a scenario .py file"
    )
    parser.add_argument(
        "--attach",
        action="store_true",
        help="Attach to an already-running sim instead of launching a headless one",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repo checkout containing gradlew (default: cwd)",
    )
    args = parser.parse_args(argv)

    module = _load_scenario(args.scenario)
    if not hasattr(module, "run"):
        print(f"SCENARIO FAIL: {args.scenario} has no run(session) function")
        return 1

    start = time.monotonic()
    try:
        with SimSession(start_sim=not args.attach, repo_root=args.repo_root) as session:
            module.run(session)
    except AssertionError as e:
        print(f"SCENARIO FAIL: {e}")
        return 1
    except Exception:  # noqa: BLE001 -- a scenario runner must catch and report anything
        traceback.print_exc()
        print("SCENARIO ERROR (unexpected exception above)")
        return 2

    print(f"SCENARIO PASS in {time.monotonic() - start:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
