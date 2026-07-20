"""TCP JSON-lines client for the local Stabilize SDK service."""

from __future__ import annotations

import json
import math
import socket
from pathlib import Path
from typing import Any, Protocol

DEFAULT_USERNAME = "admin"
DEFAULT_PASSWORD = "admin123"


class InferenceClient(Protocol):
    """Small interface shared by real and mock inference clients."""

    def __enter__(self) -> "InferenceClient": ...

    def __exit__(self, exc_type, exc, traceback) -> None: ...

    def infer(self, obs: list[float], feedback: list[float]) -> list[float]: ...


class SdkClient:
    """Client for the SDK's UTF-8 JSON-lines TCP protocol."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9000,
        timeout_s: float = 5.0,
        username: str = DEFAULT_USERNAME,
        password: str = DEFAULT_PASSWORD,
        resume: bool = True,
        logout_on_close: bool = False,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.username = username
        self.password = password
        self.resume = resume
        self.logout_on_close = logout_on_close
        self._socket: socket.socket | None = None
        self._stream = None

    def __enter__(self) -> "SdkClient":
        self.connect()
        self._login()
        self._set_resume(self.resume)
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self.logout_on_close:
            try:
                self._request({"cmd": "logout"})
            except RuntimeError:
                pass
        self.close()

    def connect(self) -> None:
        self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout_s)
        self._socket.settimeout(self.timeout_s)
        self._stream = self._socket.makefile("rwb")

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def infer(self, obs: list[float], feedback: list[float]) -> list[float]:
        response = self._request({"obs": obs, "feedback": feedback})
        self._require_status(response, "inference", {"success"})

        action = response.get("action")
        if not isinstance(action, list):
            raise RuntimeError(f"SDK inference response did not include an action list: {response}")

        try:
            return [float(value) for value in action]
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"SDK action contained non-numeric values: {action}") from exc

    def _login(self) -> None:
        response = self._request({"login": {"username": self.username, "password": self.password}})
        self._require_status(response, "login", {"ok", "success"})

    def _set_resume(self, resume: bool) -> None:
        response = self._request({"resume": resume})
        self._require_status(response, "resume", {"ok", "success"})

    def _request(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._stream is None:
            raise RuntimeError("SDK client is not connected")

        request_line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._stream.write(request_line + b"\n")
        self._stream.flush()

        response_line = self._stream.readline()
        if not response_line:
            raise RuntimeError("SDK server closed the connection without a response")

        try:
            response = json.loads(response_line.decode("utf-8"))
        except json.JSONDecodeError as exc:
            text = response_line.decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"SDK server returned invalid JSON: {text}") from exc

        if not isinstance(response, dict):
            raise RuntimeError(f"SDK server returned a non-object JSON response: {response}")
        return response

    @staticmethod
    def _require_status(response: dict[str, Any], context: str, accepted: set[str]) -> None:
        status = str(response.get("status", ""))
        if status not in accepted:
            raise RuntimeError(f"{context} failed: {json.dumps(response, ensure_ascii=False)}")


class MockSdkClient:
    """Deterministic local client for smoke tests without the SDK server."""

    def __enter__(self) -> "MockSdkClient":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None

    def infer(self, obs: list[float], feedback: list[float]) -> list[float]:
        padded_obs = (obs + [0.0] * 8)[:8]
        feedback_value = feedback[0] if feedback else 0.0

        heading_error = padded_obs[6]
        groundspeed = padded_obs[7]
        steering = _clamp(-0.65 * heading_error + 0.10 * padded_obs[5], -1.0, 1.0)
        throttle = _clamp(0.30 - 0.50 * abs(feedback_value) - 0.10 * groundspeed, -1.0, 1.0)

        auxiliary = [
            math.tanh(padded_obs[index] + 0.1 * feedback_value)
            for index in range(6)
        ]
        return [steering, throttle, *auxiliary, feedback_value, sum(padded_obs) / len(padded_obs)]


def make_inference_client(
    sdk_mode: str,
    host: str,
    port: int,
    timeout_s: float,
    username: str,
    password: str,
    resume: bool,
) -> InferenceClient:
    if sdk_mode == "mock":
        return MockSdkClient()
    if sdk_mode == "real":
        return SdkClient(
            host=host,
            port=port,
            timeout_s=timeout_s,
            username=username,
            password=password,
            resume=resume,
        )
    raise ValueError(f"Unsupported sdk_mode: {sdk_mode}")


def parse_user_info(path: Path) -> tuple[str, str]:
    """Read SDK TCP credentials from a simple userInfo-style file."""

    username = DEFAULT_USERNAME
    password = DEFAULT_PASSWORD

    if not path.exists():
        return username, password

    for line in path.read_text(encoding="utf-8").splitlines():
        normalized_line = line.replace("：", ":").strip()
        if not normalized_line or ":" not in normalized_line:
            continue

        key, value = normalized_line.split(":", 1)
        key = key.strip().lower()
        value = value.strip()

        if key in {"user_id", "username", "user", "email"}:
            username = value
        elif key in {"pwd", "password", "pass"}:
            password = value

    return username, password


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))

