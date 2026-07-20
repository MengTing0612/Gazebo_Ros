"""Shared command-line helpers for flight bridge tools."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from flight_bridge.sdk_client import DEFAULT_PASSWORD, DEFAULT_USERNAME, parse_user_info


def add_sdk_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--sdk-mode", choices=["real", "mock"], default="real")
    parser.add_argument("--host", default="127.0.0.1", help="SDK server host")
    parser.add_argument("--port", type=int, default=9000, help="SDK server port")
    parser.add_argument("--timeout", type=float, default=5.0, help="SDK socket timeout in seconds")
    parser.add_argument("--username", default=DEFAULT_USERNAME, help="SDK TCP login username")
    parser.add_argument("--password", default=DEFAULT_PASSWORD, help="SDK TCP login password")
    parser.add_argument("--user-info", type=Path, help="Optional SDK TCP login file")
    parser.add_argument("--resume", dest="resume", action="store_true", default=True)
    parser.add_argument("--no-resume", dest="resume", action="store_false")


def resolve_sdk_credentials(args: argparse.Namespace) -> tuple[str, str]:
    if args.user_info is not None:
        return parse_user_info(args.user_info)
    return args.username, args.password


def timestamped_path(directory: Path, prefix: str, suffix: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return directory / f"{prefix}_{timestamp}.{suffix}"

