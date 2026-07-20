#!/usr/bin/env python3
"""Keep PX4 SITL's GCS link alive and configure its simulated power check."""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time

from pymavlink import mavutil


GCS_SYSTEM_ID = 245
GCS_COMPONENT_ID = 190
SIM_SUPPLY_CHECK_BREAKER = 894281
PARAMETER_TIMEOUT_S = 3.0
PARAMETER_ATTEMPTS = 3


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--receive-port",
        type=int,
        default=14540,
        help="PX4 onboard MAVLink remote UDP port (default: 14540)",
    )
    parser.add_argument(
        "--wait-timeout-s",
        type=float,
        default=180.0,
        help="maximum time to wait for PX4 heartbeat (default: 180)",
    )
    return parser.parse_args()


def encode_int32_for_mavlink(value: int) -> float:
    """MAVLink PARAM_SET transports integer parameters in a float field."""
    return struct.unpack("<f", struct.pack("<i", value))[0]


def decode_int32_from_mavlink(value: float) -> int:
    return struct.unpack("<i", struct.pack("<f", value))[0]


def wait_for_px4(connection: mavutil.mavfile, timeout_s: float) -> None:
    heartbeat = connection.wait_heartbeat(timeout=timeout_s)
    if heartbeat is None:
        raise RuntimeError(f"no PX4 MAVLink heartbeat within {timeout_s:.0f} s")


def parameter_name(message: object) -> str:
    raw_name = getattr(message, "param_id")
    if isinstance(raw_name, bytes):
        return raw_name.decode("ascii").rstrip("\x00")
    return str(raw_name).rstrip("\x00")


def set_int32_parameter(connection: mavutil.mavfile, name: str, value: int) -> None:
    for attempt in range(1, PARAMETER_ATTEMPTS + 1):
        connection.mav.param_set_send(
            connection.target_system,
            connection.target_component,
            name.encode("ascii"),
            encode_int32_for_mavlink(value),
            mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        )

        deadline = time.monotonic() + PARAMETER_TIMEOUT_S
        while time.monotonic() < deadline:
            message = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if message is None or parameter_name(message) != name:
                continue
            if decode_int32_from_mavlink(message.param_value) == value:
                return

        print(
            f"waiting for {name}={value}, attempt {attempt}/{PARAMETER_ATTEMPTS}",
            file=sys.stderr,
        )

    raise RuntimeError(f"PX4 did not acknowledge {name}={value}")


def send_gcs_heartbeat(connection: mavutil.mavfile) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def run() -> int:
    args = parse_args()
    if args.receive_port <= 0 or args.receive_port > 65535:
        raise ValueError("receive port must be in 1..65535")
    if args.wait_timeout_s <= 0.0:
        raise ValueError("wait timeout must be positive")

    def stop_immediately(*_: object) -> None:
        # pymavlink socket calls can delay returning to the polling loop.
        # Exit from the signal handler so the launcher cannot leave a helper
        # process behind after the SITL session is stopped.
        raise SystemExit(0)

    signal.signal(signal.SIGINT, stop_immediately)
    signal.signal(signal.SIGTERM, stop_immediately)

    connection = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{args.receive_port}",
        source_system=GCS_SYSTEM_ID,
        source_component=GCS_COMPONENT_ID,
    )
    wait_for_px4(connection, args.wait_timeout_s)
    send_gcs_heartbeat(connection)
    set_int32_parameter(connection, "CBRK_SUPPLY_CHK", SIM_SUPPLY_CHECK_BREAKER)
    print(
        "PX4 SITL GCS heartbeat active; "
        f"CBRK_SUPPLY_CHK={SIM_SUPPLY_CHECK_BREAKER}",
        flush=True,
    )

    while True:
        time.sleep(1.0)
        send_gcs_heartbeat(connection)

    return 0


def main() -> None:
    try:
        raise SystemExit(run())
    except (RuntimeError, ValueError) as error:
        print(f"SITL GCS heartbeat failed: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
