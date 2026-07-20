"""Gazebo odometry reader for SITL truth-state checks.

The bridge normally reads MAVLink state from ArduCopter.  During simulator
episode resets, old APM inertial estimates can lag or diverge from the Gazebo
model state.  This reader keeps a lightweight subscription to Gazebo odometry
so feedback, reset triggers, and takeoff gates can use simulator truth when
running in SITL.
"""

from __future__ import annotations

import math
import subprocess
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class GazeboOdom:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    updated_at: float = 0.0

    def to_log_dict(self, home: tuple[float, float, float] | None = None) -> dict[str, float]:
        data = {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "vx": self.vx,
            "vy": self.vy,
            "vz": self.vz,
            "wx": self.wx,
            "wy": self.wy,
            "wz": self.wz,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "updated_at": self.updated_at,
        }
        if home is not None:
            data["rel_x"] = self.x - home[0]
            data["rel_y"] = self.y - home[1]
            data["rel_z"] = self.z - home[2]
        return data


class GazeboOdometryReader:
    """Background `gz topic -e` reader for `/model/x3/odometry`."""

    def __init__(self, topic: str = "/model/x3/odometry") -> None:
        self.topic = topic
        self.process: subprocess.Popen[str] | None = None
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.state: GazeboOdom | None = None
        self.running = False

    def __enter__(self) -> "GazeboOdometryReader":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()

    def start(self) -> None:
        if self.running:
            return
        command = ["gz", "topic", "-e", "-t", self.topic]
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.process = None
        self.thread = None

    def restart(self) -> None:
        self.stop()
        with self.lock:
            self.state = None
        self.start()

    def latest(self, timeout: float = 0.0) -> GazeboOdom | None:
        deadline = time.time() + max(0.0, timeout)
        while True:
            with self.lock:
                if self.state is not None:
                    return self.state
            if time.time() >= deadline:
                return None
            time.sleep(0.01)

    def _read_loop(self) -> None:
        if self.process is None or self.process.stdout is None:
            return

        path: list[str] = []
        values: dict[str, float] = {}
        for raw_line in self.process.stdout:
            if not self.running:
                break
            line = raw_line.strip()
            if line.endswith("{"):
                path.append(line[:-1].strip())
                continue
            if line == "}":
                if path:
                    path.pop()
                continue
            if ":" not in line:
                continue

            key, raw_value = line.split(":", 1)
            key = key.strip()
            raw_value = raw_value.strip()
            if key not in {"w", "x", "y", "z"}:
                continue

            current_path = ".".join(path)
            state_key = _state_key(current_path, key)
            if state_key is None:
                continue
            try:
                values[state_key] = float(raw_value)
            except ValueError:
                continue

            odom = _snapshot_from_values(values)
            if odom is not None:
                with self.lock:
                    self.state = odom


def _state_key(current_path: str, key: str) -> str | None:
    if current_path.endswith("pose.position"):
        return key if key != "w" else None
    if current_path.endswith("twist.linear"):
        return "v" + key if key != "w" else None
    if current_path.endswith("twist.angular"):
        return "w" + key if key != "w" else None
    if current_path.endswith("pose.orientation"):
        return "q" + key
    return None


def _snapshot_from_values(values: dict[str, float]) -> GazeboOdom | None:
    if not {"x", "y", "z"}.issubset(values):
        return None

    roll = pitch = yaw = 0.0
    if {"qw", "qx", "qy", "qz"}.issubset(values):
        roll, pitch, yaw = euler_from_quaternion(
            values["qw"],
            values["qx"],
            values["qy"],
            values["qz"],
        )

    return GazeboOdom(
        x=values["x"],
        y=values["y"],
        z=values["z"],
        vx=values.get("vx", 0.0),
        vy=values.get("vy", 0.0),
        vz=values.get("vz", 0.0),
        wx=values.get("wx", 0.0),
        wy=values.get("wy", 0.0),
        wz=values.get("wz", 0.0),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        updated_at=time.time(),
    )


def euler_from_quaternion(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw
