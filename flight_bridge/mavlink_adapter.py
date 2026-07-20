"""MAVLink adapter — reads vehicle state, sends control commands.

Supports two modes:
  - dry-run: deterministic simulation (Phase 0 backward-compatible)
  - sitl:    real pymavlink connection to ArduCopter SITL

Control backends (set via --control):
  - off:     read-only (no commands sent)
  - rc:      RC_CHANNELS_OVERRIDE (primary for APM 2.74b)
  - guided:  SET_POSITION_TARGET_LOCAL_NED (modern ArduCopter only)

State reading buffers last-known values per message type so that a missed
MAVLink message does not silently reset a field to zero.
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass
from typing import Any

from flight_bridge.command_types import RcCommand, VelocityCmd


# ---------------------------------------------------------------------------
# FlightState — shared vehicle state
# ---------------------------------------------------------------------------


@dataclass
class FlightState:
    """Structured vehicle state used by the bridge."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll_speed: float = 0.0
    pitch_speed: float = 0.0
    yaw_speed: float = 0.0
    heading: float = 0.0
    groundspeed: float = 0.0
    climb: float = 0.0       # vertical speed m/s (+up)
    lat: float = 0.0
    lon: float = 0.0
    relative_alt: float = 0.0
    mode: str = ""
    armed: bool = False
    battery_voltage: float = 0.0
    heading_error_deg: float = 0.0

    # Timestamps (seconds since epoch) — for freshness checks
    attitude_ts: float = 0.0
    position_ts: float = 0.0
    vfr_ts: float = 0.0
    heartbeat_ts: float = 0.0

    def fast_state_age_ms(self, now: float) -> float | None:
        """Age of oldest fast (ATT/GLOBAL/VFR) message, or None if never received."""
        ages: list[float] = []
        if self.attitude_ts > 0:
            ages.append(now - self.attitude_ts)
        if self.position_ts > 0:
            ages.append(now - self.position_ts)
        if self.vfr_ts > 0:
            ages.append(now - self.vfr_ts)
        if not ages:
            return None  # never received → stale
        return max(ages) * 1000.0

    def heartbeat_age_ms(self, now: float) -> float | None:
        """Age of last HEARTBEAT, or None if never received."""
        if self.heartbeat_ts <= 0:
            return None
        return (now - self.heartbeat_ts) * 1000.0

    def is_state_fresh(
        self, now: float,
        fast_timeout_ms: float = 200.0,
        heartbeat_timeout_ms: float = 1500.0,
    ) -> bool:
        """True if fast state (ATT/GLOBAL/VFR) and heartbeat are within limits.

        Returns False if fast state was never received or is too old.
        Heartbeat timeout is relaxed (1Hz typical).
        """
        fast_age = self.fast_state_age_ms(now)
        if fast_age is None or fast_age > fast_timeout_ms:
            return False
        hb_age = self.heartbeat_age_ms(now)
        if hb_age is not None and hb_age > heartbeat_timeout_ms:
            return False
        return True

    def to_log_dict(self) -> dict[str, float | str | bool]:
        return asdict(self)


# ---------------------------------------------------------------------------
# Command log record
# ---------------------------------------------------------------------------


@dataclass
class CommandLog:
    """Record of a command sent to the vehicle (for logging)."""

    candidate: Any = None  # RcCommand | VelocityCmd | None
    safe: Any = None
    sent: bool = False
    method: str = ""
    error: str | None = None


# ---------------------------------------------------------------------------
# MavlinkAdapter
# ---------------------------------------------------------------------------


class MavlinkAdapter:
    """Read vehicle state and send control commands.

    Phase 0 (dry-run): deterministic fake state, no real vehicle.
    Phase 1 (sitl):    pymavlink connection to ArduCopter SITL.

    State caching: once a MAVLink message type is received, its fields
    persist until overwritten by a newer message of the same type.
    """

    DEFAULT_CONNECTION = "tcp:127.0.0.1:5760"

    def __init__(
        self,
        vehicle_type: str = "copter",
        mode: str = "dry-run",
        control: str = "off",
    ) -> None:
        self.vehicle_type = vehicle_type
        self.mode = mode
        self.control = control
        self._started_at = time.monotonic()
        self._mav: Any = None
        self._connected = False
        self._cached_state: FlightState | None = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self, connection_string: str = "") -> bool:
        """Establish MAVLink connection and request data streams.

        ArduCopter 2.74b does not send data streams by default; they must
        be explicitly requested via REQUEST_DATA_STREAM.
        """
        if self.mode == "dry-run":
            return True

        conn = connection_string or self.DEFAULT_CONNECTION
        try:
            from pymavlink import mavutil
            self._mav = mavutil.mavlink_connection(conn)
            self._mav.wait_heartbeat(timeout=5)
            # Request all data streams at 10Hz (ArduCopter 2.74b requires this)
            self._mav.mav.request_data_stream_send(
                self._mav.target_system,
                self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # rate Hz
                1,   # start
            )
            self._connected = True
            return True
        except Exception:
            self._connected = False
            return False

    # ------------------------------------------------------------------
    # State reading
    # ------------------------------------------------------------------

    def read_state(self) -> FlightState:
        """Read current vehicle state with last-known-value caching."""
        if self.mode == "dry-run":
            return self._read_dry_run_state()
        return self._read_mavlink_state()

    def _read_dry_run_state(self) -> FlightState:
        """Deterministic simulated state for testing."""
        elapsed_s = time.monotonic() - self._started_at
        heading_deg = (45.0 + 3.0 * elapsed_s) % 360.0
        yaw_rad = math.radians(heading_deg)

        now = time.time()
        return FlightState(
            roll=0.02 * math.sin(0.7 * elapsed_s),
            pitch=-0.01 + 0.01 * math.cos(0.4 * elapsed_s),
            yaw=yaw_rad,
            roll_speed=0.014 * math.cos(0.7 * elapsed_s),
            pitch_speed=-0.004 * math.sin(0.4 * elapsed_s),
            yaw_speed=math.radians(3.0),
            heading=heading_deg,
            groundspeed=3.2 + 0.2 * math.sin(0.2 * elapsed_s),
            lat=47.397742,
            lon=8.545594,
            relative_alt=0.0,
            mode="STABILIZE",
            armed=True,
            battery_voltage=12000.0,
            attitude_ts=now,  # dry-run: always fresh
            position_ts=now,
            vfr_ts=now,
            heartbeat_ts=now,
        )

    def _read_mavlink_state(self) -> FlightState:
        """Read state from MAVLink with last-known-value caching.

        Each message type updates only its fields; previously-received
        fields persist across missed messages.  Timestamps enable
        staleness checks by the caller.
        """
        mav = self._mav
        if mav is None:
            return FlightState(mode="UNKNOWN", armed=False)

        now = time.time()

        # Start from cached state or fresh
        if self._cached_state is not None:
            state = FlightState(
                roll=self._cached_state.roll,
                pitch=self._cached_state.pitch,
                yaw=self._cached_state.yaw,
                roll_speed=self._cached_state.roll_speed,
                pitch_speed=self._cached_state.pitch_speed,
                yaw_speed=self._cached_state.yaw_speed,
                heading=self._cached_state.heading,
                groundspeed=self._cached_state.groundspeed,
                climb=self._cached_state.climb,
                lat=self._cached_state.lat,
                lon=self._cached_state.lon,
                relative_alt=self._cached_state.relative_alt,
                mode=self._cached_state.mode,
                armed=self._cached_state.armed,
                battery_voltage=self._cached_state.battery_voltage,
                attitude_ts=self._cached_state.attitude_ts,
                position_ts=self._cached_state.position_ts,
                vfr_ts=self._cached_state.vfr_ts,
                heartbeat_ts=self._cached_state.heartbeat_ts,
            )
        else:
            state = FlightState()

        # Drain ALL pending MAVLink messages in a single pass.
        # Using type-filtered recv_match() sequentially can miss messages
        # because pymavlink buffers are consumed per-call.  A type-agnostic
        # loop ensures every message in the buffer is dispatched.
        for _ in range(50):  # safety cap — max messages per read
            msg = mav.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "ATTITUDE":
                state.roll = float(getattr(msg, "roll", 0.0))
                state.pitch = float(getattr(msg, "pitch", 0.0))
                state.yaw = float(getattr(msg, "yaw", 0.0))
                state.roll_speed = float(getattr(msg, "rollspeed", 0.0))
                state.pitch_speed = float(getattr(msg, "pitchspeed", 0.0))
                state.yaw_speed = float(getattr(msg, "yawspeed", 0.0))
                state.attitude_ts = now
            elif msg_type == "GLOBAL_POSITION_INT":
                state.lat = float(getattr(msg, "lat", 0)) / 1e7
                state.lon = float(getattr(msg, "lon", 0)) / 1e7
                state.relative_alt = float(getattr(msg, "relative_alt", 0)) / 1000.0
                state.position_ts = now
            elif msg_type == "VFR_HUD":
                state.heading = float(getattr(msg, "heading", 0.0))
                state.groundspeed = float(getattr(msg, "groundspeed", 0.0))
                state.climb = float(getattr(msg, "climb", 0.0))
                state.vfr_ts = now
            elif msg_type == "HEARTBEAT":
                state.armed = (getattr(msg, "base_mode", 0) & 0x80) != 0
                state.mode = _decode_copter_mode(getattr(msg, "custom_mode", 0))
                state.heartbeat_ts = now

        self._cached_state = state
        return state

    # ------------------------------------------------------------------
    # Control commands — dispatch based on self.control
    # ------------------------------------------------------------------

    def send_rc_cmd(self, cmd: RcCommand) -> CommandLog:
        """Send RC override (requires --control rc)."""
        log = CommandLog(candidate=cmd, safe=cmd)

        if self.control != "rc":
            log.sent = False
            log.error = f"send_rc_cmd requires --control rc, got {self.control}"
            return log

        if self.mode == "dry-run":
            log.sent = False
            return log

        try:
            self.send_rc_override(cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw)
            log.sent = True
            log.method = "RC_CHANNELS_OVERRIDE"
        except Exception as exc:
            log.sent = False
            log.error = str(exc)

        return log

    def send_velocity_cmd(self, cmd: VelocityCmd) -> CommandLog:
        """Send guided velocity (requires --control guided)."""
        log = CommandLog(candidate=cmd, safe=cmd)

        if self.control != "guided":
            log.sent = False
            log.error = f"send_velocity_cmd requires --control guided, got {self.control}"
            return log

        if self.mode == "dry-run":
            log.sent = False
            return log

        try:
            self._send_guided_velocity(cmd)
            log.sent = True
            log.method = "SET_POSITION_TARGET_LOCAL_NED"
        except Exception as exc:
            log.sent = False
            log.error = str(exc)

        return log

    def send_rc_override(
        self, roll: int = 1500, pitch: int = 1500,
        throttle: int = 1000, yaw: int = 1500,
        ch5: int = 0, ch6: int = 0, ch7: int = 0, ch8: int = 0,
    ) -> None:
        """Send RC_CHANNELS_OVERRIDE.

        Args:
            roll, pitch, yaw: 1000-2000 PWM (1500 = neutral)
            throttle: 1000-2000 PWM
            ch5-ch8: 0 = don't override, otherwise PWM value
        """
        mav = self._mav
        if mav is None or self.mode == "dry-run":
            return
        mav.mav.rc_channels_override_send(
            mav.target_system, mav.target_component,
            roll, pitch, throttle, yaw, ch5, ch6, ch7, ch8,
        )

    def set_mode(self, mode: str) -> bool:
        """Switch flight mode (STABILIZE, ALT_HOLD, LOITER, GUIDED, etc.).

        Uses MAVLINK_MSG_ID_SET_MODE (msg #11) which APM 2.74b expects.
        pymavlink's set_mode() sends MAV_CMD_DO_SET_MODE which is not supported
        by this APM version.
        """
        mav = self._mav
        if mav is None or self.mode == "dry-run":
            return False
        mode_id = _copter_mode_id(mode)
        if mode_id is None:
            return False
        # APM 2.74b GCS_Mavlink.pde:1266 checks base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        # base_mode is a uint8 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1)
        mav.mav.set_mode_send(
            mav.target_system,
            1,  # base_mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            mode_id,
        )
        return True

    def arm_vehicle(self, method: str = "both", timeout_s: float = 8.0) -> bool:
        """Arm the copter with throttle low.

        APM 2.74b SITL is sometimes happier with the old rudder gesture than
        with MAV_CMD_COMPONENT_ARM_DISARM, so the default tries both.
        """
        if self.mode == "dry-run":
            return True
        if self._cached_state is not None and self._cached_state.armed:
            return True

        method = method.lower()
        if method in {"command", "both"} and self._arm_with_command(timeout_s):
            return True
        if method in {"rudder", "both"} and self._arm_with_rudder(timeout_s):
            return True
        return False

    def emergency_stop(self) -> None:
        """Immediate hover fallback via RC neutral + zero throttle."""
        if self.mode == "dry-run":
            return
        self.send_rc_override(1500, 1500, 1000, 1500)

    # ------------------------------------------------------------------
    # Internal: guided velocity
    # ------------------------------------------------------------------

    def _arm_with_command(self, timeout_s: float) -> bool:
        mav = self._mav
        if mav is None:
            return False

        from pymavlink import mavutil

        self.send_rc_override(1500, 1500, 1000, 1500)
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        return self._wait_until_armed(timeout_s=min(timeout_s, 5.0))

    def _arm_with_rudder(self, timeout_s: float) -> bool:
        deadline = time.time() + min(timeout_s, 5.0)
        while time.time() < deadline:
            self.send_rc_override(1500, 1500, 1000, 2000)
            time.sleep(0.05)
        self.send_rc_override(1500, 1500, 1000, 1500)
        return self._wait_until_armed(timeout_s=min(timeout_s, 6.0))

    def _wait_until_armed(self, timeout_s: float) -> bool:
        mav = self._mav
        if mav is None:
            return False

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            if msg.get_type() != "HEARTBEAT":
                continue
            armed = (getattr(msg, "base_mode", 0) & 0x80) != 0
            if armed:
                if self._cached_state is not None:
                    self._cached_state.armed = True
                    self._cached_state.heartbeat_ts = time.time()
                return True
        return False

    def _send_guided_velocity(self, cmd: VelocityCmd) -> None:
        """Send SET_POSITION_TARGET_LOCAL_NED with velocity mask."""
        mav = self._mav
        if mav is None:
            raise RuntimeError("Not connected")

        type_mask = 0b0000_0111_1000_0111  # ignore pos, accel; use vel + yaw_rate
        mav.mav.set_position_target_local_ned_send(
            0,
            mav.target_system,
            mav.target_component,
            1,                          # MAV_FRAME_LOCAL_NED
            type_mask,
            0.0, 0.0, 0.0,            # position (ignored)
            cmd.vx, cmd.vy, cmd.vz,    # velocity m/s NED
            0.0, 0.0, 0.0,            # acceleration (ignored)
            0.0,                       # yaw (ignored)
            cmd.yaw_rate,              # yaw_rate rad/s
        )


# ---------------------------------------------------------------------------
# Copter mode helpers
# ---------------------------------------------------------------------------

_COPTER_MODES: dict[int, str] = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    8: "POSITION",
    9: "LAND",
    10: "OF_LOITER",
}


def _decode_copter_mode(custom_mode: int) -> str:
    return _COPTER_MODES.get(custom_mode, f"MODE_{custom_mode}")


def _copter_mode_id(mode: str) -> int | None:
    mode_upper = mode.upper()
    for mid, name in _COPTER_MODES.items():
        if name == mode_upper:
            return mid
    return None
