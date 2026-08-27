"""Auto smoke scenario: select the RIGHT auto and let it run.

Asserts the robot actually drives (pose moves) and that RobotManager leaves
IDLE at some point during the routine (i.e. the auto state machine issues
requests). Prints the observed state timeline for the run log.

Run: uv run --package sim-agent sim-agent right_auto
"""

import time
from pathlib import Path

from sim_agent.session import ESTIMATED_POSE, ROBOT_MANAGER_STATE, SimSession

AUTO_DURATION_SECONDS = 12.0


def run(session: SimSession) -> None:
    session.start_recording(
        {
            ROBOT_MANAGER_STATE: "string",
            ESTIMATED_POSE: "pose2d",
            "/Robot/Autos/State": "string",
        }
    )
    try:
        session.enable_auto("RIGHT")
        print("[scenario] autonomous enabled with RIGHT auto")

        session.wait_pose_moved(1.0, timeout=20.0)
        print("[scenario] robot moved under auto control")

        # Watch the rest of the routine and collect the RobotManager states we see.
        states_seen: set[str] = set()
        deadline = time.monotonic() + AUTO_DURATION_SECONDS
        while time.monotonic() < deadline:
            state = session.get(ROBOT_MANAGER_STATE, "string")
            if state:
                states_seen.add(state)
            time.sleep(0.05)
        print(f"[scenario] RobotManager states seen: {sorted(states_seen)}")
        assert len(states_seen) > 1, (
            f"RobotManager never left IDLE during auto: {states_seen}"
        )

        session.disable()
    finally:
        out = Path(session.runs_dir()) / f"right_auto-{int(time.time())}.json"
        session.stop_recording(out)
