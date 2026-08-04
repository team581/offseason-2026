"""SimSession: agent-in-the-loop control of a WPILib simulator process.

Connects to the robot's NT4 server on localhost, drives the robot via the
``/Agent`` topics consumed by ``com.team581.sim.AgentSimShim``, and observes
``/Robot`` topics published by DogLog. Can also manage the simulator process
itself (headless ``./gradlew comp-bot:simulateJava``).
"""

import json
import math
import os
import signal
import subprocess
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, Literal, Self

import ntcore

from sim_agent.structs import (
    Pose2dTuple,
    decode_chassis_speeds,
    decode_pose2d,
)

ValueKind = Literal[
    "bool", "double", "string", "double_array", "pose2d", "chassis_speeds"
]
Getter = Callable[[], Any]

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 3
AXIS_RIGHT_X = 4
AXIS_RIGHT_Y = 5

BUTTON_NAMES = (
    "a",
    "b",
    "x",
    "y",
    "lb",
    "rb",
    "back",
    "start",
    "left_stick",
    "right_stick",
)

ROBOT_MANAGER_STATE = "/Robot/RobotManager/State"
SWERVE_STATE = "/Robot/Swerve/State"
ESTIMATED_POSE = "/Robot/Localization/EstimatedPose"
AUTO_CHOOSER_SELECTED = "/SmartDashboard/Autos/SelectedAuto/selected"


class SimSession:
    """Context manager for one agent-driven sim run.

    Args:
        start_sim: If true, launch ``./gradlew comp-bot:simulateJava`` headless and kill it on
            exit. If false, attach to an already-running sim (e.g. one a human is watching).
        repo_root: Repo checkout containing ``gradlew``; defaults to the current directory.
        runs_dir: Where sim stdout and recordings go; defaults to
            ``<repo_root>/comp-bot/logs/agent-runs`` (gitignored via ``logs/``).
        connect_timeout: Seconds to wait for the NT4 connection and shim handshake.
    """

    def __init__(
        self,
        *,
        start_sim: bool = True,
        repo_root: Path | None = None,
        runs_dir: Path | None = None,
        connect_timeout: float = 240.0,
    ) -> None:
        self._repo_root = (repo_root or Path.cwd()).resolve()
        self._runs_dir = (
            runs_dir or self._repo_root / "comp-bot" / "logs" / "agent-runs"
        )
        self._start_sim = start_sim
        self._connect_timeout = connect_timeout

        self._proc: subprocess.Popen[bytes] | None = None
        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._start_time = time.monotonic()

        # /Agent publishers
        self._enabled_pub: ntcore.BooleanPublisher | None = None
        self._axes_pub: ntcore.DoubleArrayPublisher | None = None
        self._buttons_pub: ntcore.BooleanArrayPublisher | None = None
        self._ds_enabled_pub: ntcore.BooleanPublisher | None = None
        self._mode_pub: ntcore.StringPublisher | None = None
        self._alliance_pub: ntcore.StringPublisher | None = None
        self._auto_selected_pub: ntcore.StringPublisher | None = None

        self._axes = [0.0] * 6
        self._buttons = [False] * 10

        self._recorder_stop: threading.Event | None = None
        self._recorder_thread: threading.Thread | None = None
        self._recorder_samples: dict[str, list[list[Any]]] = {}

    # ------------------------------------------------------------------ lifecycle

    def __enter__(self) -> Self:
        self._runs_dir.mkdir(parents=True, exist_ok=True)
        if self._start_sim:
            self._launch_sim()
        self._connect_nt()
        self._publish_defaults()
        self._wait_handshake()
        self._start_time = time.monotonic()
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        try:
            self.zero_inputs()
            self.disable()
            self._set_agent_enabled(False)
            time.sleep(0.2)
        finally:
            self._inst.stopClient()
            self._kill_sim()

    def _launch_sim(self) -> None:
        stdout_path = self._runs_dir / f"sim-stdout-{int(time.time())}.log"
        env = os.environ.copy()
        env["SIM_HEADLESS"] = "1"
        # --no-daemon so the sim JVM is our child and dies with the process group.
        cmd = ["./gradlew", "--no-daemon", "--console=plain", "comp-bot:simulateJava"]
        print(f"[sim_agent] starting sim: {' '.join(cmd)} (stdout -> {stdout_path})")
        stdout_file = open(stdout_path, "wb")  # noqa: SIM115 -- closed by Popen teardown
        self._proc = subprocess.Popen(
            cmd,
            cwd=self._repo_root,
            env=env,
            stdout=stdout_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def _kill_sim(self) -> None:
        if self._proc is None:
            return
        print("[sim_agent] stopping sim")
        try:
            os.killpg(self._proc.pid, signal.SIGTERM)
            self._proc.wait(timeout=15)
        except ProcessLookupError, subprocess.TimeoutExpired:
            try:
                os.killpg(self._proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        # Safety net in case the sim JVM escaped the process group.
        subprocess.run(
            ["pkill", "-f", r"comp-bot-.*\.jar"], check=False, capture_output=True
        )
        self._proc = None

    def _connect_nt(self) -> None:
        self._inst.setServer("127.0.0.1")
        self._inst.startClient4("sim-agent")
        deadline = time.monotonic() + self._connect_timeout
        while not self._inst.isConnected():
            if time.monotonic() > deadline:
                raise TimeoutError(
                    "Timed out connecting to the robot NT4 server on localhost:5810. "
                    "Is the sim running? (managed sim stdout is in the runs dir)"
                )
            if self._proc is not None and self._proc.poll() is not None:
                raise RuntimeError(
                    f"Sim process exited early (code {self._proc.returncode}); "
                    "check sim stdout in the runs dir"
                )
            time.sleep(0.5)
        print("[sim_agent] connected to NT4 server")

    def _publish_defaults(self) -> None:
        agent = self._inst.getTable("/Agent")
        self._enabled_pub = agent.getBooleanTopic("Enabled").publish()
        self._axes_pub = agent.getDoubleArrayTopic("Joystick/Axes").publish()
        self._buttons_pub = agent.getBooleanArrayTopic("Joystick/Buttons").publish()
        self._ds_enabled_pub = agent.getBooleanTopic("DriverStation/Enabled").publish()
        self._mode_pub = agent.getStringTopic("DriverStation/Mode").publish()
        self._alliance_pub = agent.getStringTopic(
            "DriverStation/AllianceStation"
        ).publish()
        self._auto_selected_pub = (
            self._inst.getTable("/SmartDashboard/Autos/SelectedAuto")
            .getStringTopic("selected")
            .publish()
        )

        self._set_agent_enabled(True)
        self._publish_axes()
        self._publish_buttons()
        self._ds_enabled_pub.set(False)
        self._mode_pub.set("teleop")
        self._alliance_pub.set("")

    def _wait_handshake(self) -> None:
        active_sub = (
            self._inst.getTable("/Agent")
            .getBooleanTopic("Status/Active")
            .subscribe(False)
        )
        deadline = time.monotonic() + self._connect_timeout
        while not active_sub.get():
            if time.monotonic() > deadline:
                raise TimeoutError(
                    "NT connected but /Agent/Status/Active never went true. "
                    "Is the robot running an AgentSimShim build?"
                )
            time.sleep(0.2)
        print("[sim_agent] AgentSimShim handshake OK")

    def _set_agent_enabled(self, value: bool) -> None:
        assert self._enabled_pub is not None
        self._enabled_pub.set(value)

    def _publish_axes(self) -> None:
        assert self._axes_pub is not None
        # ty: ignore[invalid-argument-type] -- robotpy stubs use invariant list params
        self._axes_pub.set(self._axes)

    def _publish_buttons(self) -> None:
        assert self._buttons_pub is not None
        # ty: ignore[invalid-argument-type] -- robotpy stubs use invariant list params
        self._buttons_pub.set(self._buttons)

    # ------------------------------------------------------------------ time

    def time(self) -> float:
        """Seconds since the session handshake completed."""
        return time.monotonic() - self._start_time

    # ------------------------------------------------------------------ driver station

    def enable_teleop(self) -> None:
        assert self._mode_pub is not None and self._ds_enabled_pub is not None
        self._mode_pub.set("teleop")
        self._ds_enabled_pub.set(True)

    def enable_test(self) -> None:
        assert self._mode_pub is not None and self._ds_enabled_pub is not None
        self._mode_pub.set("test")
        self._ds_enabled_pub.set(True)

    def enable_auto(self, auto_name: str) -> None:
        """Select an auto on the chooser, then enable autonomous."""
        assert self._mode_pub is not None and self._ds_enabled_pub is not None
        self.select_auto(auto_name)
        # The robot rebuilds the auto and resets the pose to its start while
        # disabled; wait for that to finish before enabling.
        self.wait_pose_settled()
        self._mode_pub.set("autonomous")
        self._ds_enabled_pub.set(True)
        self.wait_robot_enabled()

    def disable(self) -> None:
        assert self._ds_enabled_pub is not None
        self._ds_enabled_pub.set(False)

    def wait_robot_enabled(self, timeout: float = 10.0) -> None:
        """Wait until the robot reports itself enabled via the shim.

        Robot-side NT/DS setup lags a fresh connection by ~0.5-1s, so writes
        made right after the handshake take that long to take effect. Use this
        after ``enable_*`` when a scenario needs the enable to have landed
        (e.g. before trusting that no more disabled pose resets can happen).
        """
        self.wait_for(
            "/Agent/Status/RobotEnabled",
            "bool",
            lambda enabled: enabled,
            timeout=timeout,
            description="robot enabled (shim echo)",
        )

    def select_auto(self, auto_name: str) -> None:
        """Write the chooser selection and wait for the robot to acknowledge it.

        Robot-side NT setup for the SendableChooser can lag the session handshake
        by ~1s, and the robot only polls the chooser while disabled. A
        fire-and-forget write can therefore land after autonomous is already
        enabled and never take effect. Re-assert the write until the robot
        echoes the selection on the chooser's ``active`` topic.
        """
        assert self._auto_selected_pub is not None
        active_sub = (
            self._inst.getTable("/SmartDashboard/Autos/SelectedAuto")
            .getStringTopic("active")
            .subscribe("")
        )
        deadline = time.monotonic() + 5.0
        while True:
            self._auto_selected_pub.set(auto_name)
            if active_sub.get() == auto_name:
                return
            if time.monotonic() > deadline:
                raise AssertionError(
                    f"Auto selection {auto_name!r} not acknowledged by the robot; "
                    f"chooser active={active_sub.get()!r}"
                )
            time.sleep(0.1)

    def set_alliance(self, station: str) -> None:
        """Set alliance station, e.g. ``"Red1"`` or ``"Blue2"``."""
        assert self._alliance_pub is not None
        self._alliance_pub.set(station)

    # ------------------------------------------------------------------ joystick

    def set_sticks(
        self,
        *,
        left_x: float | None = None,
        left_y: float | None = None,
        right_x: float | None = None,
        left_trigger: float | None = None,
        right_trigger: float | None = None,
    ) -> None:
        """Set stick/trigger axes using WPILib XboxController conventions.

        All values are raw axis values in [-1, 1]: ``left_y=-1`` is full forward
        (stick pushed up), triggers range 0..1 (boolean event fires at >= 0.5).
        ``None`` leaves the axis unchanged.
        """
        updates = {
            AXIS_LEFT_X: left_x,
            AXIS_LEFT_Y: left_y,
            AXIS_RIGHT_X: right_x,
            AXIS_LEFT_TRIGGER: left_trigger,
            AXIS_RIGHT_TRIGGER: right_trigger,
        }
        for axis, value in updates.items():
            if value is not None:
                self._axes[axis] = max(-1.0, min(1.0, value))
        self._publish_axes()

    def set_buttons(self, **buttons: bool) -> None:
        """Set buttons by name: a, b, x, y, lb, rb, back, start, left_stick, right_stick.

        Note: button bindings only fire while the robot is enabled, and LT/RT are
        axes (use ``set_sticks(left_trigger=...)``), not buttons.
        """
        for name, pressed in buttons.items():
            self._buttons[BUTTON_NAMES.index(name)] = pressed
        self._publish_buttons()

    def zero_inputs(self) -> None:
        """Center all sticks and release all buttons."""
        self._axes = [0.0] * 6
        self._buttons = [False] * 10
        if self._axes_pub is not None:
            self._publish_axes()
        if self._buttons_pub is not None:
            self._publish_buttons()

    # ------------------------------------------------------------------ observation

    def getter(self, topic_path: str, kind: ValueKind) -> Getter:
        """Build a pollable getter for a /Robot (or any) topic.

        ``pose2d``/``chassis_speeds`` decode DogLog struct payloads; they return
        ``None`` until a well-formed value arrives.
        """
        table_path, _, key = topic_path.rpartition("/")
        table = self._inst.getTable(table_path or "/")
        match kind:
            case "bool":
                sub = table.getBooleanTopic(key).subscribe(False)
                return sub.get
            case "double":
                sub = table.getDoubleTopic(key).subscribe(0.0)
                return sub.get
            case "string":
                sub = table.getStringTopic(key).subscribe("")
                return sub.get
            case "double_array":
                sub = table.getDoubleArrayTopic(key).subscribe([])
                return lambda: list(sub.get())
            case "pose2d":
                sub = table.getRawTopic(key).subscribe("struct:Pose2d", b"")
                return lambda: decode_pose2d(sub.get())
            case "chassis_speeds":
                sub = table.getRawTopic(key).subscribe("struct:ChassisSpeeds", b"")
                return lambda: decode_chassis_speeds(sub.get())

    def get(self, topic_path: str, kind: ValueKind) -> Any:
        """One-shot read of a topic."""
        return self.getter(topic_path, kind)()

    def wait_for(
        self,
        topic_path: str,
        kind: ValueKind,
        predicate: Callable[[Any], bool],
        timeout: float = 10.0,
        description: str | None = None,
    ) -> Any:
        """Poll a topic at 50 Hz until ``predicate(value)`` is true; return the value.

        Raises AssertionError on timeout (scenarios treat that as a failure).
        """
        get = self.getter(topic_path, kind)
        deadline = time.monotonic() + timeout
        value = get()
        while not predicate(value):
            if time.monotonic() > deadline:
                raise AssertionError(
                    f"Timed out ({timeout}s) waiting for {topic_path}: "
                    f"{description or 'predicate'}; last value={value!r}"
                )
            time.sleep(0.02)
            value = get()
        return value

    def wait_state(self, *states: str, timeout: float = 10.0) -> str:
        """Wait until RobotManager/State is one of ``states``; return it."""
        return self.wait_for(
            ROBOT_MANAGER_STATE,
            "string",
            lambda s: s in states,
            timeout=timeout,
            description=f"state in {states}",
        )

    def wait_pose_moved(self, meters: float, timeout: float = 15.0) -> Pose2dTuple:
        """Wait until EstimatedPose has moved ``meters`` from its current position."""
        start = self.get(ESTIMATED_POSE, "pose2d")
        if start is None:
            start = self.wait_for(
                ESTIMATED_POSE,
                "pose2d",
                lambda p: p is not None,
                timeout=5.0,
                description="first pose sample",
            )

        def moved(pose: Pose2dTuple | None) -> bool:
            return (
                pose is not None
                and math.hypot(pose[0] - start[0], pose[1] - start[1]) >= meters
            )

        return self.wait_for(
            ESTIMATED_POSE,
            "pose2d",
            moved,
            timeout=timeout,
            description=f"pose moved >= {meters} m",
        )

    def wait_pose_settled(
        self,
        tolerance_m: float = 0.02,
        window_s: float = 0.4,
        timeout: float = 5.0,
    ) -> Pose2dTuple:
        """Wait until EstimatedPose stops teleporting and holds still.

        On sim boot (and on auto selection) the robot's disabled pose reset
        teleports EstimatedPose to the selected auto's starting pose up to ~1s
        after the session handshake. A reference pose sampled before that
        teleport makes ``wait_pose_moved`` pass without the robot driving.
        Poll until the pose moves less than ``tolerance_m`` across ``window_s``.
        """
        deadline = time.monotonic() + timeout
        last = self.wait_for(
            ESTIMATED_POSE,
            "pose2d",
            lambda p: p is not None,
            timeout=5.0,
            description="first pose sample",
        )
        while True:
            time.sleep(window_s)
            current = self.get(ESTIMATED_POSE, "pose2d")
            if (
                current is not None
                and math.hypot(current[0] - last[0], current[1] - last[1])
                <= tolerance_m
            ):
                return current
            if current is not None:
                last = current
            if time.monotonic() > deadline:
                raise AssertionError(
                    f"EstimatedPose never settled within {timeout}s; "
                    f"last value={last!r}"
                )

    # ------------------------------------------------------------------ recording

    def start_recording(self, topics: dict[str, ValueKind], hz: float = 50.0) -> None:
        """Sample topics in a background thread until ``stop_recording``."""
        if self._recorder_thread is not None:
            raise RuntimeError("already recording")
        getters = {name: self.getter(name, kind) for name, kind in topics.items()}
        self._recorder_samples = {name: [] for name in topics}
        self._recorder_stop = threading.Event()

        def loop() -> None:
            assert self._recorder_stop is not None
            period = 1.0 / hz
            while not self._recorder_stop.is_set():
                t = self.time()
                for name, get in getters.items():
                    value = get()
                    if isinstance(value, tuple):
                        value = list(value)
                    self._recorder_samples[name].append([round(t, 4), value])
                time.sleep(period)

        self._recorder_thread = threading.Thread(target=loop, daemon=True)
        self._recorder_thread.start()

    def stop_recording(self, path: Path) -> Path:
        """Stop recording and write samples to JSON."""
        if self._recorder_stop is None or self._recorder_thread is None:
            raise RuntimeError("not recording")
        self._recorder_stop.set()
        self._recorder_thread.join(timeout=2.0)
        self._recorder_thread = None
        self._recorder_stop = None
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"topics": self._recorder_samples}, indent=1))
        print(f"[sim_agent] recording saved to {path}")
        return path

    def runs_dir(self) -> Path:
        return self._runs_dir
