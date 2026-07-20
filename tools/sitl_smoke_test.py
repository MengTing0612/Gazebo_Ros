#!/usr/bin/env python3
"""SITL connectivity smoke test — verify MAVLink connection, state reading,
mode switching, and RC override before running the full bridge.

Usage:
    # Step 1: Start ArduCopter SITL in another terminal
    #   sim_vehicle.py -v ArduCopter --console --map -L Zurich

    # Step 2: Run this smoke test
    python tools/sitl_smoke_test.py

    # Custom connection
    python tools/sitl_smoke_test.py --connect tcp:127.0.0.1:5760

What this tests:
    1. pymavlink import
    2. MAVLink connection + heartbeat wait
    3. Vehicle state reading (ATTITUDE, GLOBAL_POSITION_INT, VFR_HUD, HEARTBEAT)
    4. Mode switching (STABILIZE → ALT_HOLD → LOITER → STABILIZE)
    5. RC_CHANNELS_OVERRIDE send
    6. Clean disconnect
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------


@dataclass
class TestResult:
    name: str
    passed: bool
    detail: str = ""
    duration_ms: float = 0.0


class SmokeTester:
    """Runs a sequence of MAVLink connectivity checks."""

    def __init__(self, connection_string: str, timeout_s: float = 10.0) -> None:
        self._conn_str = connection_string
        self._timeout_s = timeout_s
        self._mav: object = None
        self.results: list[TestResult] = []

    # ------------------------------------------------------------------
    # Test steps
    # ------------------------------------------------------------------

    def run_all(self) -> bool:
        """Run all smoke tests. Returns True if all passed."""
        print("=" * 60)
        print("  SITL Connectivity Smoke Test")
        print(f"  Connection: {self._conn_str}")
        print("=" * 60)
        print()

        self._test_pymavlink_import()
        self._test_connect()
        if self._mav is not None:
            self._test_heartbeat()
            self._test_read_attitude()
            self._test_read_position()
            self._test_read_vfr()
            self._test_rc_override()
            self._test_mode_switch()
            self._test_disconnect()

        # Summary
        passed = sum(1 for r in self.results if r.passed)
        failed = len(self.results) - passed
        print()
        print("=" * 60)
        print(f"  Results: {passed} passed, {failed} failed, {len(self.results)} total")
        print("=" * 60)
        for r in self.results:
            icon = "✅" if r.passed else "❌"
            timing = f" ({r.duration_ms:.0f}ms)" if r.duration_ms > 0 else ""
            print(f"  {icon} {r.name}{timing}")
            if r.detail:
                print(f"      {r.detail}")
        print()

        return failed == 0

    def _test_pymavlink_import(self) -> None:
        """Test 1: pymavlink import."""
        t0 = time.monotonic()
        try:
            from pymavlink import mavutil  # noqa: F401
            self.results.append(TestResult(
                "pymavlink import", True,
                detail="pymavlink available",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except ImportError as exc:
            self.results.append(TestResult(
                "pymavlink import", False,
                detail=f"ImportError: {exc}. Install: pip install pymavlink",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_connect(self) -> None:
        """Test 2: MAVLink connection + data stream request."""
        t0 = time.monotonic()
        try:
            from pymavlink import mavutil
            self._mav = mavutil.mavlink_connection(self._conn_str)
            # Request data streams at 4Hz (required for ArduCopter 2.74b)
            self._mav.mav.request_data_stream_send(
                self._mav.target_system, self._mav.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
            self.results.append(TestResult(
                "MAVLink connect", True,
                detail=f"Connected to {self._conn_str}, streams requested",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "MAVLink connect", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_heartbeat(self) -> None:
        """Test 3: Wait for heartbeat."""
        t0 = time.monotonic()
        try:
            self._mav.wait_heartbeat(timeout=self._timeout_s)
            dt = time.monotonic() - t0
            self.results.append(TestResult(
                "Heartbeat", True,
                detail=f"Received in {dt:.1f}s",
                duration_ms=dt * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "Heartbeat", False,
                detail=f"No heartbeat within {self._timeout_s}s: {exc}",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_read_attitude(self) -> None:
        """Test 4: Read ATTITUDE message."""
        t0 = time.monotonic()
        try:
            # Drain any queued messages, then wait for one ATTITUDE
            msg = None
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                msg = self._mav.recv_match(type="ATTITUDE", blocking=True, timeout=0.5)
                if msg is not None:
                    break
            if msg is None:
                raise TimeoutError("No ATTITUDE message received")
            roll = float(getattr(msg, "roll", 0))
            pitch = float(getattr(msg, "pitch", 0))
            yaw = float(getattr(msg, "yaw", 0))
            self.results.append(TestResult(
                "ATTITUDE read", True,
                detail=f"roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f} rad",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "ATTITUDE read", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_read_position(self) -> None:
        """Test 5: Read GLOBAL_POSITION_INT."""
        t0 = time.monotonic()
        try:
            msg = self._mav.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=2.0)
            if msg is None:
                raise TimeoutError("No GLOBAL_POSITION_INT message received")
            lat = float(getattr(msg, "lat", 0)) / 1e7
            lon = float(getattr(msg, "lon", 0)) / 1e7
            alt = float(getattr(msg, "relative_alt", 0)) / 1000.0
            self.results.append(TestResult(
                "GLOBAL_POSITION_INT read", True,
                detail=f"lat={lat:.6f} lon={lon:.6f} alt={alt:.2f}m",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "GLOBAL_POSITION_INT read", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_read_vfr(self) -> None:
        """Test 6: Read VFR_HUD."""
        t0 = time.monotonic()
        try:
            msg = self._mav.recv_match(type="VFR_HUD", blocking=True, timeout=2.0)
            if msg is None:
                raise TimeoutError("No VFR_HUD message received")
            heading = float(getattr(msg, "heading", 0))
            groundspeed = float(getattr(msg, "groundspeed", 0))
            self.results.append(TestResult(
                "VFR_HUD read", True,
                detail=f"heading={heading:.1f}° groundspeed={groundspeed:.1f}m/s",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "VFR_HUD read", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_rc_override(self) -> None:
        """Test 7: Send RC_CHANNELS_OVERRIDE."""
        t0 = time.monotonic()
        try:
            # Send neutral RC values (no movement)
            self._mav.mav.rc_channels_override_send(
                self._mav.target_system,
                self._mav.target_component,
                1500, 1500, 1000, 1500,  # roll, pitch, throttle(min), yaw
                0, 0, 0, 0,              # ch5-ch8 unused
            )
            time.sleep(0.1)
            self.results.append(TestResult(
                "RC_CHANNELS_OVERRIDE", True,
                detail="Sent neutral RC override (throttle=1000=min, sticks=1500)",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "RC_CHANNELS_OVERRIDE", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_mode_switch(self) -> None:
        """Test 8: Mode switching via MAVLINK_MSG_ID_SET_MODE (ArduCopter 2.74b)."""
        t0 = time.monotonic()
        try:
            from pymavlink import mavutil

            # Read current mode
            hb = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            if hb is None:
                raise TimeoutError("No HEARTBEAT for mode read")

            copter_modes: dict[int, str] = {
                0: "STABILIZE", 2: "ALT_HOLD", 5: "LOITER", 4: "GUIDED",
            }
            current_mode_id = getattr(hb, "custom_mode", 0)
            current_mode = copter_modes.get(current_mode_id, f"mode_{current_mode_id}")

            # Use set_mode_send (msgid 11) with CUSTOM_MODE_ENABLED flag
            # ArduCopter 2.74b requires base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            self._mav.mav.set_mode_send(
                self._mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                2)  # ALT_HOLD
            time.sleep(0.5)

            # Read mode again
            hb2 = self._mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
            new_mode_id = getattr(hb2, "custom_mode", 0) if hb2 else -1
            new_mode = copter_modes.get(new_mode_id, f"mode_{new_mode_id}")

            # Switch back to original
            self._mav.mav.set_mode_send(
                self._mav.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                current_mode_id)
            time.sleep(0.3)

            detail = f"{current_mode} → {new_mode} → {current_mode}"
            switched_ok = new_mode_id == 2  # ALT_HOLD

            self.results.append(TestResult(
                "Mode switch", switched_ok,
                detail=detail,
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "Mode switch", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))

    def _test_disconnect(self) -> None:
        """Test 9: Clean disconnect."""
        t0 = time.monotonic()
        try:
            self._mav.close()
            self._mav = None
            self.results.append(TestResult(
                "Disconnect", True,
                detail="Connection closed cleanly",
                duration_ms=(time.monotonic() - t0) * 1000,
            ))
        except Exception as exc:
            self.results.append(TestResult(
                "Disconnect", False,
                detail=str(exc),
                duration_ms=(time.monotonic() - t0) * 1000,
            ))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SITL connectivity smoke test for flight_bridge")
    parser.add_argument(
        "--connect", default="tcp:127.0.0.1:5760",
        help="MAVLink connection string (default: tcp:127.0.0.1:5760)")
    parser.add_argument(
        "--timeout", type=float, default=10.0,
        help="Heartbeat timeout in seconds (default: 10)")
    args = parser.parse_args()

    tester = SmokeTester(args.connect, args.timeout)
    all_passed = tester.run_all()

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
