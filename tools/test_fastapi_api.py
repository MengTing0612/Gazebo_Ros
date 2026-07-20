"""Smoke tests for the FastAPI flight control service."""

from __future__ import annotations

import time

from flight_bridge.api import ControlStartRequest, FlightControlService, StepControlRequest


def test_health_and_status() -> None:
    service = FlightControlService()
    status = service.status()
    assert status.running is False
    assert status.frame == 0


def test_dry_run_start_rejects_duplicate_and_stops(tmp_path) -> None:
    service = FlightControlService()

    response = service.start(
        ControlStartRequest(
            mode="dry-run",
            sdk_mode="mock",
            rate_hz=80.0,
            iterations=20,
            log_file=tmp_path / "api.jsonl",
        )
    )
    try:
        assert response.writes_enabled is False

        try:
            service.start(ControlStartRequest(mode="dry-run", sdk_mode="mock"))
        except RuntimeError as exc:
            assert "already running" in str(exc)
        else:
            raise AssertionError("duplicate start should fail")

        deadline = time.time() + 2.0
        latest = None
        while time.time() < deadline:
            latest = service.status()
            if latest.frame >= 2:
                break
            time.sleep(0.03)

        assert latest is not None
        assert latest.latest_obs is not None
        assert len(latest.latest_obs) == 16
        assert latest.latest_action is not None
        assert latest.latest_safe_command is not None
    finally:
        service.stop()


def test_step_returns_safe_rc_command() -> None:
    service = FlightControlService()

    response = service.step(
        StepControlRequest(
            sdk_mode="mock",
            z_err_m=0.8,
            altitude_m=0.2,
            armed=True,
            mode="STABILIZE",
        )
    )
    assert len(response.obs) == 16
    assert len(response.feedback) == 1
    assert len(response.action) == 10
    assert len(response.safe_command) == 4
    assert response.sent is False
