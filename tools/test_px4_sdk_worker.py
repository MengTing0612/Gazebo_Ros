#!/usr/bin/env python3
"""Verify that the ROS 2 controller can consume SDK output without blocking."""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE = ROOT / "ros2_ws" / "src" / "micoair_hover"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROS_PACKAGE))

from micoair_hover.sdk_worker import SdkWorker


def main() -> int:
    worker = SdkWorker(
        mode="mock",
        host="127.0.0.1",
        port=9000,
        timeout_s=0.1,
        username="unused",
        password="unused",
        resume=True,
    )
    worker.start()
    try:
        worker.submit([0.0] * 16, [0.5])
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            result = worker.latest()
            if result.received_monotonic_s > 0.0:
                if result.error is not None:
                    raise AssertionError(result.error)
                if len(result.action) < 4:
                    raise AssertionError(f"expected at least four SDK actions, got {result.action}")
                print("PX4 SDK worker smoke test passed")
                return 0
            time.sleep(0.01)
        raise AssertionError("mock SDK worker did not produce an action within one second")
    finally:
        worker.close()


if __name__ == "__main__":
    raise SystemExit(main())
