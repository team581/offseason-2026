"""Teleop smoke scenario: drive, rotate, and score via the agent gamepad.

Covers the full teleop input path end to end:
  enable teleop -> IDLE -> drive forward (left stick) -> pose moves
  -> rotate (right stick) -> heading changes -> hold RT -> score/feed state
  -> release RT -> back to IDLE.

Run: uv run --package sim-agent sim-agent drive_and_score
"""

import time
from pathlib import Path

from sim_agent.session import (
    ESTIMATED_POSE,
    ROBOT_MANAGER_STATE,
    SWERVE_STATE,
    SimSession,
)

SCORE_OR_FEED_STATES = ("PREPARE_SCORE", "SCORE", "PREPARE_FEED", "FEED")


def run(session: SimSession) -> None:
    session.start_recording(
        {
            ROBOT_MANAGER_STATE: "string",
            SWERVE_STATE: "string",
            ESTIMATED_POSE: "pose2d",
        }
    )
    try:
        session.enable_teleop()
        session.wait_state("IDLE", timeout=5.0)
        print("[scenario] teleop enabled, state IDLE")

        # The disabled pose reset teleports the robot to the auto start pose
        # ~0.5-1s after boot; it can only fire while disabled, so once the shim
        # reports the robot enabled and the pose holds still, wait_pose_moved
        # below measures real driving, not the teleport.
        session.wait_robot_enabled()
        session.wait_pose_settled()

        # Drive forward at 60% stick (magnitude shaping squares it to ~36% speed).
        session.set_sticks(left_y=-0.6)
        pose = session.wait_pose_moved(1.0, timeout=10.0)
        session.set_sticks(left_y=0.0)
        print(f"[scenario] translation OK, pose={pose}")

        # Rotate in place and check the heading changes.
        _, _, start_heading = pose
        session.set_sticks(right_x=0.6)
        time.sleep(1.0)
        session.set_sticks(right_x=0.0)
        _, _, end_heading = session.wait_for(
            ESTIMATED_POSE,
            "pose2d",
            lambda p: p is not None,
            timeout=3.0,
            description="pose after rotation",
        )
        delta = abs((end_heading - start_heading + 180.0) % 360.0 - 180.0)
        assert delta > 15.0, f"heading barely changed while rotating: {delta:.1f} deg"
        print(f"[scenario] rotation OK, heading delta={delta:.1f} deg")

        # Hold RT: score-or-feed request (which one depends on field zone).
        session.set_sticks(right_trigger=1.0)
        state = session.wait_state(*SCORE_OR_FEED_STATES, timeout=5.0)
        print(f"[scenario] RT press -> {state}")

        # Release RT: back to IDLE.
        session.set_sticks(right_trigger=0.0)
        session.wait_state("IDLE", timeout=5.0)
        print("[scenario] RT release -> IDLE")

        session.disable()
    finally:
        out = Path(session.runs_dir()) / f"drive_and_score-{int(time.time())}.json"
        session.stop_recording(out)
