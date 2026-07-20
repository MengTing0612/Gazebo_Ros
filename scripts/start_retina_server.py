#!/usr/bin/env python3
"""Start retina-server-x86_64 and optionally answer its login prompts."""

from __future__ import annotations

import argparse
import errno
import os
import pty
import select
import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class WebCredentials:
    username: str
    password: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--server",
        type=Path,
        default=Path("examples/retina-server-x86_64"),
        help="Path to the retina server executable",
    )
    parser.add_argument(
        "--user-info",
        type=Path,
        default=Path("examples/userInfo.txt"),
        help="Web login credentials file, using user_id/pwd keys",
    )
    parser.add_argument(
        "--no-auto-login",
        action="store_true",
        help="Start the server without answering Email/Password prompts",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.server.exists():
        raise SystemExit(f"Server executable does not exist: {args.server}")

    credentials = None if args.no_auto_login else read_web_credentials(args.user_info)
    return run_server(args.server, credentials)


def read_web_credentials(path: Path) -> WebCredentials:
    if not path.exists():
        raise SystemExit(f"Credential file does not exist: {path}")

    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        normalized_line = line.replace("：", ":").strip()
        if not normalized_line or ":" not in normalized_line:
            continue

        key, value = normalized_line.split(":", 1)
        values[key.strip().lower()] = value.strip()

    username = values.get("user_id") or values.get("email") or values.get("username")
    password = values.get("pwd") or values.get("password") or values.get("pass")
    if not username or not password:
        raise SystemExit(f"Credential file must contain user_id and pwd: {path}")

    return WebCredentials(username=username, password=password)


def run_server(server_path: Path, credentials: WebCredentials | None) -> int:
    server_path = server_path.resolve()
    server_cwd = server_path.parent
    master_fd, slave_fd = pty.openpty()
    process = subprocess.Popen(
        [f"./{server_path.name}"],
        cwd=server_cwd,
        stdin=slave_fd,
        stdout=slave_fd,
        stderr=slave_fd,
        close_fds=True,
    )
    os.close(slave_fd)

    responded_email = False
    responded_password = False
    output_buffer = ""

    def terminate_child(signum, _frame) -> None:
        if process.poll() is None:
            process.send_signal(signum)

    signal.signal(signal.SIGINT, terminate_child)
    signal.signal(signal.SIGTERM, terminate_child)

    try:
        while True:
            if process.poll() is not None:
                drain_available_output(master_fd)
                return int(process.returncode or 0)

            readable, _, _ = select.select([master_fd], [], [], 0.1)
            if not readable:
                continue

            chunk = read_pty(master_fd)
            if chunk is None:
                return int(process.wait())
            if not chunk:
                continue

            text = chunk.decode("utf-8", errors="replace")
            sys.stdout.write(redact_password_echo(text, credentials))
            sys.stdout.flush()

            output_buffer = (output_buffer + text)[-512:]
            if credentials is None:
                continue

            if not responded_email and "Email or phone:" in output_buffer:
                write_line(master_fd, credentials.username)
                responded_email = True
                output_buffer = ""
                continue

            if not responded_password and "Password:" in output_buffer:
                write_line(master_fd, credentials.password)
                responded_password = True
                output_buffer = ""
    finally:
        os.close(master_fd)


def write_line(fd: int, text: str) -> None:
    os.write(fd, (text + "\n").encode("utf-8"))


def read_pty(fd: int) -> bytes | None:
    try:
        return os.read(fd, 4096)
    except OSError as exc:
        if exc.errno == errno.EIO:
            return None
        raise


def redact_password_echo(text: str, credentials: WebCredentials | None) -> str:
    if credentials is None:
        return text
    return text.replace(credentials.password, "<redacted>")


def drain_available_output(fd: int) -> None:
    while True:
        readable, _, _ = select.select([fd], [], [], 0)
        if not readable:
            return
        chunk = read_pty(fd)
        if chunk is None:
            return
        if not chunk:
            return
        sys.stdout.write(chunk.decode("utf-8", errors="replace"))
        sys.stdout.flush()


if __name__ == "__main__":
    raise SystemExit(main())
