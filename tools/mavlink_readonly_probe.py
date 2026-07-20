#!/usr/bin/env python3
"""Read-only MAVLink connection probe for APM/Pixhawk style devices."""

from __future__ import annotations

import argparse
from pathlib import Path

from pymavlink import mavutil
from serial.tools import list_ports


DEFAULT_BAUDS = [115200, 57600, 38400]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--device", action="append", help="Serial device to test, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, action="append", help="Baud rate to test")
    parser.add_argument("--udp", action="append", help="UDP endpoint to test, e.g. udpin:0.0.0.0:14550")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    devices = args.device or discover_serial_devices()
    bauds = args.baud or DEFAULT_BAUDS
    udp_endpoints = args.udp or ["udpin:0.0.0.0:14550", "udpin:0.0.0.0:14540"]

    print("serial candidates:")
    if devices:
        for device in devices:
            print(f"  {device}")
    else:
        print("  none")

    found = False
    for device in devices:
        for baud in bauds:
            found = probe_endpoint(device, baud=baud, timeout=args.timeout) or found

    for endpoint in udp_endpoints:
        found = probe_endpoint(endpoint, baud=None, timeout=args.timeout) or found

    return 0 if found else 1


def discover_serial_devices() -> list[str]:
    devices = []
    for port in list_ports.comports():
        if port.device.startswith(("/dev/ttyACM", "/dev/ttyUSB")):
            devices.append(port.device)

    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for path in sorted(Path("/").glob(pattern.removeprefix("/"))):
            device = "/" + str(path)
            if device not in devices:
                devices.append(device)
    return devices


def probe_endpoint(endpoint: str, baud: int | None, timeout: float) -> bool:
    label = endpoint if baud is None else f"{endpoint} @ {baud}"
    print(f"checking {label} ...")
    try:
        connection = mavutil.mavlink_connection(
            endpoint,
            baud=baud if baud is not None else 115200,
            autoreconnect=False,
            robust_parsing=True,
            source_system=255,
        )
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        if message is None:
            print("  no heartbeat")
            return False

        print(
            "  HEARTBEAT "
            f"system={connection.target_system} "
            f"component={connection.target_component} "
            f"type={message.type} "
            f"autopilot={message.autopilot} "
            f"base_mode={message.base_mode} "
            f"custom_mode={message.custom_mode}"
        )
        return True
    except Exception as exc:
        print(f"  {type(exc).__name__}: {exc}")
        return False
    finally:
        try:
            connection.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
