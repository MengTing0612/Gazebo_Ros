"""Phase 0 dry-run loop for SDK inference without control output."""

from __future__ import annotations

import argparse
import socket
import time

from flight_bridge.cli import default_log_path, validate_common_args
from flight_bridge.cli_common import resolve_sdk_credentials
from flight_bridge.jsonl_logger import JsonlLogger
from flight_bridge.mavlink_adapter import MavlinkAdapter
from flight_bridge.observation_builder import ObservationBuilder, build_feedback, wrap_deg
from flight_bridge.run_summary import (
    ActionSummary,
    format_frame_summary,
    should_print_summary,
    sleep_remaining,
)
from flight_bridge.sdk_client import make_inference_client


def run_phase0(args: argparse.Namespace) -> int:
    validate_phase0_args(args)
    username, password = resolve_sdk_credentials(args)

    log_file = args.log_file or default_log_path(args.log_dir, "phase0")
    adapter = MavlinkAdapter(vehicle_type="rover", mode="dry-run", control="off")
    obs_builder = ObservationBuilder()
    action_summary = ActionSummary()

    print(f"phase0 mode=dry-run control=off sdk_mode={args.sdk_mode} log={log_file}")

    try:
        inference_client = make_inference_client(
            sdk_mode=args.sdk_mode, host=args.host, port=args.port,
            timeout_s=args.timeout, username=username, password=password,
            resume=args.resume,
        )
        with inference_client as sdk, JsonlLogger(log_file) as logger:
            run_phase0_loop(args, adapter, obs_builder, sdk, logger, action_summary)
    except ConnectionRefusedError as exc:
        raise SystemExit(
            f"Cannot connect to SDK server at {args.host}:{args.port}. "
            "Start ./examples/retina-server-x86_64 first, or use --sdk-mode mock."
        ) from exc
    except socket.timeout as exc:
        raise SystemExit(
            f"Timed out talking to SDK server at {args.host}:{args.port}. "
            "Check the server or increase --timeout."
        ) from exc

    print("final " + action_summary.format_line())
    return 0


def validate_phase0_args(args: argparse.Namespace) -> None:
    if args.mode != "dry-run":
        raise SystemExit("Phase 0 only implements --mode dry-run")
    if args.control != "off":
        raise SystemExit("Phase 0 refuses write control; use --control off")
    validate_common_args(args)


def run_phase0_loop(
    args: argparse.Namespace,
    adapter: MavlinkAdapter,
    obs_builder: ObservationBuilder,
    sdk,
    logger: JsonlLogger,
    action_summary: ActionSummary,
) -> None:
    frame_index = 0
    period_s = 1.0 / args.rate_hz

    while args.iterations <= 0 or frame_index < args.iterations:
        frame_started_at = time.perf_counter()
        state = adapter.read_state()
        state.heading_error_deg = wrap_deg(args.target_heading - state.heading)
        obs = obs_builder.build(state)
        feedback = build_feedback(state)

        action: list[float] = []
        latency_ms = 0.0
        error_text: str | None = None

        try:
            infer_started_at = time.perf_counter()
            action = sdk.infer(obs, feedback)
            latency_ms = (time.perf_counter() - infer_started_at) * 1000.0
            action_summary.update(action, latency_ms)
        except RuntimeError as exc:
            error_text = str(exc)

        logger.write({
            "ts": time.time(), "frame": frame_index,
            "mode": args.mode, "control": args.control,
            "vehicle_mode": state.mode, "armed": state.armed,
            "state": state.to_log_dict(), "obs": obs, "feedback": feedback,
            "action": action, "latency_ms": latency_ms,
            "candidate_command": None, "safe_command": None, "sent": False,
            "error": error_text,
        })

        if error_text is not None:
            raise RuntimeError(error_text)

        if should_print_summary(frame_index, args.summary_every):
            print(format_frame_summary(frame_index, state.mode, obs, action, latency_ms))

        frame_index += 1
        sleep_remaining(frame_started_at, period_s)
