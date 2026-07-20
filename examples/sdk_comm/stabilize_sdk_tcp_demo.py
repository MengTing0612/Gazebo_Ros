#!/usr/bin/env python3
"""Minimal TCP JSON-line client for the Stabilize-ai inference SDK."""

from __future__ import annotations

import argparse
import json
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULT_USERNAME = "admin"
DEFAULT_PASSWORD = "admin123"


@dataclass
class SdkResponse:
    raw: dict[str, Any]

    @property
    def status(self) -> str:
        return str(self.raw.get("status", ""))

    def require_ok(self, context: str) -> None:
        if self.status not in {"ok", "success"}:
            raise RuntimeError(f"{context} failed: {json.dumps(self.raw, ensure_ascii=False)}")


class StabilizeSdkClient:
    """Client for the SDK's line-delimited JSON TCP protocol."""

    def __init__(self, host: str, port: int, timeout_s: float) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self._socket: socket.socket | None = None
        self._stream = None

    def __enter__(self) -> "StabilizeSdkClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
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

    def login(self, username: str, password: str) -> SdkResponse:
        return self.request({"login": {"username": username, "password": password}})

    def set_resume(self, resume: bool) -> SdkResponse:
        return self.request({"resume": resume})

    def infer(self, obs: list[float], feedback: list[float]) -> SdkResponse:
        return self.request({"obs": obs, "feedback": feedback})

    def logout(self) -> SdkResponse:
        return self.request({"cmd": "logout"})

    def request(self, payload: dict[str, Any]) -> SdkResponse:
        if self._stream is None:
            raise RuntimeError("SDK client is not connected")

        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._stream.write(line + b"\n")
        self._stream.flush()

        response_line = self._stream.readline()
        if not response_line:
            raise RuntimeError("SDK server closed the connection without a response")

        try:
            return SdkResponse(json.loads(response_line.decode("utf-8")))
        except json.JSONDecodeError as exc:
            text = response_line.decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"SDK server returned invalid JSON: {text}") from exc


def parse_float_list(text: str) -> list[float]:
    if not text.strip():
        return []
    return [float(item.strip()) for item in text.split(",")]


def parse_user_info(path: Path) -> tuple[str, str]:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="SDK server host")
    parser.add_argument("--port", type=int, default=9000, help="SDK server port")
    parser.add_argument(
        "--user-info",
        type=Path,
        help="Optional path to a TCP login userInfo file; overrides the built-in local credentials",
    )
    parser.add_argument("--username", help="SDK TCP login username")
    parser.add_argument("--password", help="SDK TCP login password")
    parser.add_argument("--resume", action="store_true", help="Resume the previous policy session")
    parser.add_argument("--obs", default="", help="Comma-separated observation vector")
    parser.add_argument("--obs-len", type=int, default=8, help="Zero observation length if --obs is empty")
    parser.add_argument("--feedback", default="0.0", help="Comma-separated feedback vector")
    parser.add_argument("--timeout", type=float, default=10.0, help="Socket timeout in seconds")
    parser.add_argument("--logout", action="store_true", help="Send logout before closing")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    obs = parse_float_list(args.obs) if args.obs else [0.0] * args.obs_len
    feedback = parse_float_list(args.feedback)
    username = DEFAULT_USERNAME
    password = DEFAULT_PASSWORD

    if args.user_info is not None:
        username, password = parse_user_info(args.user_info)

    if args.username is not None:
        username = args.username
    if args.password is not None:
        password = args.password

    try:
        with StabilizeSdkClient(args.host, args.port, args.timeout) as client:
            login_response = client.login(username, password)
            login_response.require_ok("login")
            print("login:", json.dumps(login_response.raw, ensure_ascii=False))

            resume_response = client.set_resume(args.resume)
            resume_response.require_ok("resume")
            print("resume:", json.dumps(resume_response.raw, ensure_ascii=False))

            infer_response = client.infer(obs, feedback)
            infer_response.require_ok("inference")
            print("inference:", json.dumps(infer_response.raw, ensure_ascii=False))

            action = infer_response.raw.get("action")
            if action is not None:
                print("action:", action)

            if args.logout:
                logout_response = client.logout()
                print("logout:", json.dumps(logout_response.raw, ensure_ascii=False))
    except ConnectionRefusedError as exc:
        raise SystemExit(
            f"无法连接 Stabilize SDK 服务端 {args.host}:{args.port}。\n"
            "通常原因是 retina-server-x86_64/retina-server-arm 没有启动，"
            "或者服务端监听端口和 --port 不一致。\n"
            "请先在另一个终端启动 SDK 服务端，例如：\n"
            "  chmod +x ./retina-server-x86_64\n"
            "  ./retina-server-x86_64\n"
            "然后保持服务端运行，再重新执行本 demo。"
        ) from exc
    except socket.timeout as exc:
        raise SystemExit(
            f"连接或等待 Stabilize SDK 服务端超时：{args.host}:{args.port}。"
            "请确认服务端已启动、端口正确，并适当增大 --timeout。"
        ) from exc
    except OSError as exc:
        raise SystemExit(
            f"访问 Stabilize SDK 服务端失败：{args.host}:{args.port}，错误：{exc}"
        ) from exc


if __name__ == "__main__":
    main()
