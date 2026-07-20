"""FastAPI service wrapper for the flight bridge control loop.

The API keeps the CLI untouched while exposing the Phase 1 RC control path as
a service.  It is deliberately dry-run by default; MAVLink writes only happen
when a start request explicitly selects ``mode="sitl"``.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Literal

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, Field, field_validator

from flight_bridge.action_adapter import ActionAdapter
from flight_bridge.cli import default_log_path
from flight_bridge.command_types import RcCommand, ShieldInfo
from flight_bridge.config_loader import load_config
from flight_bridge.gazebo_odometry import GazeboOdom, GazeboOdometryReader
from flight_bridge.jsonl_logger import JsonlLogger
from flight_bridge.main import (
    _baseline_rc_for_state,
    _copter_state_to_log,
    _flightstate_to_copterstate,
    _protect_low_altitude_attitude,
    _protect_low_altitude_throttle,
)
from flight_bridge.mavlink_adapter import CommandLog, MavlinkAdapter
from flight_bridge.run_summary import ActionSummary, sleep_remaining
from flight_bridge.safety_shield import SafetyShield
from flight_bridge.sdk_client import (
    DEFAULT_PASSWORD,
    DEFAULT_USERNAME,
    InferenceClient,
    make_inference_client,
)
from flight_bridge.state_builder import CopterState, CopterStateBuilder, HoverFeedbackBuilder


ControlMode = Literal["dry-run", "sitl"]
SdkMode = Literal["mock", "real"]
RcControlMode = Literal["residual", "sdk-direct", "baseline-only"]
AdapterMode = Literal["front-4", "learned"]


class RcCommandModel(BaseModel):
    roll_pwm: int
    pitch_pwm: int
    throttle_pwm: int
    yaw_pwm: int


class ShieldInfoModel(BaseModel):
    saturated: bool = False
    safety_triggered: bool = False
    clipped_fields: list[str] = Field(default_factory=list)
    filtered: bool = False
    in_deadband: bool = False


class CopterStateInput(BaseModel):
    x_err_m: float = 0.0
    y_err_m: float = 0.0
    z_err_m: float = 0.0
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_err_rad: float = 0.0
    p_rad_s: float = 0.0
    q_rad_s: float = 0.0
    r_rad_s: float = 0.0
    altitude_m: float = 0.0
    armed: bool = False
    mode: str = ""

    def to_copter_state(self) -> CopterState:
        return CopterState(
            x_err=self.x_err_m,
            y_err=self.y_err_m,
            z_err=self.z_err_m,
            vx=self.vx_mps,
            vy=self.vy_mps,
            vz=self.vz_mps,
            roll=self.roll_rad,
            pitch=self.pitch_rad,
            yaw_err=self.yaw_err_rad,
            p=self.p_rad_s,
            q=self.q_rad_s,
            r=self.r_rad_s,
            altitude_m=self.altitude_m,
            armed=self.armed,
            mode=self.mode,
        )


class ControlStartRequest(BaseModel):
    mode: ControlMode = "dry-run"
    connect: str = ""
    sdk_mode: SdkMode = "mock"
    host: str = "127.0.0.1"
    port: int = 9000
    timeout_s: float = 5.0
    username: str = DEFAULT_USERNAME
    password: str = DEFAULT_PASSWORD
    resume: bool = True
    rate_hz: float = 20.0
    iterations: int = 0
    target_alt_m: float = 3.0
    target_heading_deg: float = 0.0
    target_x_m: float | None = None
    target_y_m: float | None = None
    config: Path = Path("flight_bridge/config.yaml")
    log_file: Path | None = None
    log_dir: Path = Path("logs")
    use_real_limits: bool = False
    rc_control_mode: RcControlMode = "residual"
    adapter_mode: AdapterMode = "front-4"
    gaia_adapter_config: Path = Path("flight_bridge/gaia_adapter.yaml")
    use_gazebo_truth: bool = False
    gazebo_odom_topic: str = "/model/x3/odometry"
    sdk_direct_level_until_alt_m: float = 0.70

    @field_validator("rate_hz")
    @classmethod
    def _validate_rate_hz(cls, value: float) -> float:
        if value <= 0.0:
            raise ValueError("rate_hz must be positive")
        return value

    @field_validator("iterations")
    @classmethod
    def _validate_iterations(cls, value: int) -> int:
        if value < 0:
            raise ValueError("iterations must be >= 0")
        return value

    @field_validator("timeout_s")
    @classmethod
    def _validate_timeout_s(cls, value: float) -> float:
        if value <= 0.0:
            raise ValueError("timeout_s must be positive")
        return value


class ControlStartResponse(BaseModel):
    running: bool
    mode: ControlMode
    control: Literal["rc"] = "rc"
    sdk_mode: SdkMode
    rate_hz: float
    log_file: str
    writes_enabled: bool


class ControlStopResponse(BaseModel):
    running: bool
    stopped: bool
    latest_error: str | None = None


class ControlStatusResponse(BaseModel):
    running: bool
    frame: int
    mode: str | None = None
    control: str | None = None
    sdk_mode: str | None = None
    rate_hz: float | None = None
    log_file: str | None = None
    latest_state: dict[str, Any] | None = None
    latest_copter_state: dict[str, Any] | None = None
    latest_obs: list[float] | None = None
    latest_feedback: list[float] | None = None
    latest_action: list[float] | None = None
    latest_safe_command: list[int] | None = None
    latest_safe_command_struct: RcCommandModel | None = None
    latest_error: str | None = None
    summary: str


class StepControlRequest(CopterStateInput):
    sdk_mode: SdkMode = "mock"
    host: str = "127.0.0.1"
    port: int = 9000
    timeout_s: float = 5.0
    username: str = DEFAULT_USERNAME
    password: str = DEFAULT_PASSWORD
    resume: bool = True
    config: Path = Path("flight_bridge/config.yaml")
    use_real_limits: bool = False
    rc_control_mode: RcControlMode = "residual"
    adapter_mode: AdapterMode = "front-4"
    gaia_adapter_config: Path = Path("flight_bridge/gaia_adapter.yaml")
    send: bool = False


class StepControlResponse(BaseModel):
    obs: list[float]
    feedback: list[float]
    feedback_diagnostics: dict[str, Any]
    action: list[float]
    latency_ms: float
    baseline_command: list[int]
    candidate_command: list[int]
    candidate_command_struct: RcCommandModel
    safe_command: list[int]
    safe_command_struct: RcCommandModel
    shield: ShieldInfoModel
    sent: bool
    cmd_method: str
    error: str | None = None


class _ControlSession:
    def __init__(
        self,
        request: ControlStartRequest,
        config: Any,
        log_file: Path,
        adapter: MavlinkAdapter,
        sdk: InferenceClient,
        logger: JsonlLogger,
    ) -> None:
        self.request = request
        self.config = config
        self.log_file = log_file
        self.adapter = adapter
        self.sdk = sdk
        self.logger = logger
        self.state_builder = CopterStateBuilder()
        self.feedback_builder = HoverFeedbackBuilder(config.feedback)
        self.action_adapter = ActionAdapter(config.control, config.limits, config.copter)
        self.shield = SafetyShield(
            config.safety,
            config.limits,
            config.filter,
            hover_throttle_pwm=config.copter.hover_throttle + 1000,
        )
        self.action_summary = ActionSummary()
        self.started_at_monotonic = time.monotonic()
        self.frame_lock = threading.RLock()
        self.home_lat: float | None = None
        self.home_lon: float | None = None
        self.gazebo_home: tuple[float, float, float] | None = None
        self.last_rc_delta: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.last_shield_info = ShieldInfo()
        self.gazebo_reader: GazeboOdometryReader | None = None

    @property
    def writes_enabled(self) -> bool:
        return self.request.mode == "sitl"

    def load_optional_adapter(self) -> None:
        if self.request.adapter_mode != "learned":
            return
        if self.request.gaia_adapter_config.exists():
            self.action_adapter.load_learned(self.request.gaia_adapter_config)

    def start_gazebo_reader(self) -> None:
        if self.request.mode != "sitl" or not self.request.use_gazebo_truth:
            return
        self.gazebo_reader = GazeboOdometryReader(self.request.gazebo_odom_topic)
        self.gazebo_reader.start()

    def close(self) -> None:
        if self.gazebo_reader is not None:
            self.gazebo_reader.stop()
            self.gazebo_reader = None
        self.logger.__exit__(None, None, None)
        self.sdk.__exit__(None, None, None)


class FlightControlService:
    """Threaded service around the RC control path."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._session: _ControlSession | None = None
        self._session_info: dict[str, Any] | None = None
        self._latest_frame: dict[str, Any] | None = None
        self._latest_error: str | None = None
        self._frame_index = 0

    def start(self, request: ControlStartRequest) -> ControlStartResponse:
        with self._lock:
            if self.is_running_locked():
                raise RuntimeError("control loop is already running")

            session = self._create_session(request)
            self._stop_event = threading.Event()
            self._session = session
            self._session_info = _session_info(session)
            self._latest_error = None
            self._latest_frame = None
            self._frame_index = 0

            thread = threading.Thread(
                target=self._run_loop,
                args=(session, self._stop_event),
                name="flight-bridge-control",
                daemon=True,
            )
            self._thread = thread
            thread.start()

            return ControlStartResponse(
                running=True,
                mode=request.mode,
                sdk_mode=request.sdk_mode,
                rate_hz=request.rate_hz,
                log_file=str(session.log_file),
                writes_enabled=session.writes_enabled,
            )

    def stop(self, timeout_s: float = 3.0) -> ControlStopResponse:
        with self._lock:
            thread = self._thread
            session = self._session
            was_running = thread is not None and thread.is_alive()
            if not was_running:
                return ControlStopResponse(
                    running=False,
                    stopped=False,
                    latest_error=self._latest_error,
                )
            self._stop_event.set()

        thread.join(timeout=timeout_s)
        if thread.is_alive() and session is not None and session.writes_enabled:
            session.adapter.emergency_stop()

        with self._lock:
            return ControlStopResponse(
                running=thread.is_alive(),
                stopped=not thread.is_alive(),
                latest_error=self._latest_error,
            )

    def status(self) -> ControlStatusResponse:
        with self._lock:
            session = self._session
            session_info = dict(self._session_info) if self._session_info else None
            latest = dict(self._latest_frame) if self._latest_frame is not None else None
            running = self.is_running_locked()
            latest_error = self._latest_error
            frame_index = self._frame_index

            if latest is None:
                return ControlStatusResponse(
                    running=running,
                    frame=frame_index,
                    mode=_status_value(session, session_info, "mode"),
                    control=_status_value(session, session_info, "control"),
                    sdk_mode=_status_value(session, session_info, "sdk_mode"),
                    rate_hz=_status_value(session, session_info, "rate_hz"),
                    log_file=_status_value(session, session_info, "log_file"),
                    latest_error=latest_error,
                    summary=session.action_summary.format_line() if session else "frames=0",
                )

            safe_command = latest.get("safe_command")
            return ControlStatusResponse(
                running=running,
                frame=int(latest.get("frame", frame_index)),
                mode=str(latest.get("mode", "")),
                control=str(latest.get("control", "")),
                sdk_mode=_status_value(session, session_info, "sdk_mode"),
                rate_hz=_status_value(session, session_info, "rate_hz"),
                log_file=_status_value(session, session_info, "log_file"),
                latest_state=latest.get("state"),
                latest_copter_state=latest.get("copter_state"),
                latest_obs=latest.get("obs"),
                latest_feedback=latest.get("feedback"),
                latest_action=latest.get("action"),
                latest_safe_command=safe_command,
                latest_safe_command_struct=_rc_model_from_list(safe_command),
                latest_error=latest_error or latest.get("error"),
                summary=(
                    session.action_summary.format_line()
                    if session
                    else str(session_info.get("summary", "frames=0") if session_info else "frames=0")
                ),
            )

    def step(self, request: StepControlRequest) -> StepControlResponse:
        copter_state = request.to_copter_state()
        with self._lock:
            session = self._session if self.is_running_locked() else None

        if session is not None:
            return self._run_step_with_session(session, copter_state, request.send)
        return self._run_step_with_temporary_context(request, copter_state)

    def is_running_locked(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _create_session(self, request: ControlStartRequest) -> _ControlSession:
        config = load_config(request.config, use_real=request.use_real_limits)
        log_file = request.log_file or default_log_path(request.log_dir, "api")
        adapter = MavlinkAdapter(vehicle_type="copter", mode=request.mode, control="rc")

        if request.mode == "sitl" and not adapter.connect(request.connect):
            raise ConnectionError(
                f"failed to connect to MAVLink at {request.connect or adapter.DEFAULT_CONNECTION}"
            )

        sdk = make_inference_client(
            sdk_mode=request.sdk_mode,
            host=request.host,
            port=request.port,
            timeout_s=request.timeout_s,
            username=request.username,
            password=request.password,
            resume=request.resume,
        )
        logger = JsonlLogger(log_file)

        try:
            sdk.__enter__()
            logger.__enter__()
            session = _ControlSession(request, config, log_file, adapter, sdk, logger)
            session.load_optional_adapter()
            session.start_gazebo_reader()
            return session
        except Exception:
            logger.__exit__(None, None, None)
            sdk.__exit__(None, None, None)
            raise

    def _run_loop(self, session: _ControlSession, stop_event: threading.Event) -> None:
        request = session.request
        period_s = 1.0 / request.rate_hz
        try:
            while not stop_event.is_set():
                if request.iterations > 0 and self._frame_index >= request.iterations:
                    break
                frame_started_at = time.perf_counter()
                try:
                    with session.frame_lock:
                        record = self._run_background_frame(session, self._frame_index)
                except Exception as exc:
                    self._latest_error = str(exc)
                    break
                with self._lock:
                    self._latest_frame = record
                    self._frame_index = int(record["frame"]) + 1
                    self._session_info = _session_info(session)
                sleep_remaining(frame_started_at, period_s)
        finally:
            if session.writes_enabled:
                for _ in range(8):
                    session.adapter.emergency_stop()
                    time.sleep(0.03)
            session.close()
            with self._lock:
                self._session_info = _session_info(session)
                self._session = None
                self._thread = None

    def _run_background_frame(
        self, session: _ControlSession, frame_index: int,
    ) -> dict[str, Any]:
        request = session.request
        raw_state = session.adapter.read_state()
        now = time.time()

        if request.mode == "sitl" and session.home_lat is None and raw_state.lat != 0:
            session.home_lat = raw_state.lat
            session.home_lon = raw_state.lon

        gazebo_odom = self._latest_gazebo_odom(session)
        if gazebo_odom is not None and session.gazebo_home is None:
            session.gazebo_home = (gazebo_odom.x, gazebo_odom.y, gazebo_odom.z)

        state_age_ms = raw_state.fast_state_age_ms(now)
        if state_age_ms is None:
            state_age_ms = 9999.0
        state_fresh = (
            request.mode == "dry-run"
            or raw_state.is_state_fresh(now)
        )
        gazebo_age_ms = _gazebo_age_ms(gazebo_odom, now)
        gazebo_fresh = (
            not request.use_gazebo_truth
            or gazebo_age_ms is not None and gazebo_age_ms <= session.config.safety.state_age_max_ms
        )

        copter_state = _flightstate_to_copterstate(
            raw_state,
            request.target_heading_deg,
            target_alt_m=request.target_alt_m,
            target_x_m=request.target_x_m,
            target_y_m=request.target_y_m,
            elapsed_s=time.monotonic() - session.started_at_monotonic,
            is_dry_run=request.mode == "dry-run",
            home_lat=session.home_lat,
            home_lon=session.home_lon,
            gazebo_odom=gazebo_odom,
            gazebo_home=session.gazebo_home,
        )
        result = self._infer_and_send_rc(
            session=session,
            copter_state=copter_state,
            state_age_ms=state_age_ms,
            state_fresh=state_fresh,
            gazebo_fresh=gazebo_fresh,
            allow_send=True,
        )

        record = self._build_log_record(
            session=session,
            frame_index=frame_index,
            raw_state=raw_state.to_log_dict(),
            vehicle_mode=raw_state.mode,
            armed=raw_state.armed,
            copter_state=copter_state,
            result=result,
            state_age_ms=state_age_ms,
            gazebo_age_ms=gazebo_age_ms,
            gazebo_odom=gazebo_odom,
        )
        session.logger.write(record)
        return record

    def _infer_and_send_rc(
        self,
        session: _ControlSession,
        copter_state: CopterState,
        state_age_ms: float,
        state_fresh: bool,
        gazebo_fresh: bool,
        allow_send: bool,
    ) -> dict[str, Any]:
        baseline_rc = _baseline_rc_for_state(session.action_adapter, copter_state)
        obs = session.state_builder.build_hover_obs(
            copter_state,
            session.last_rc_delta,
            rc_mode=True,
        )
        feedback = session.feedback_builder.build(
            copter_state,
            shield_info=session.last_shield_info,
            last_cmd=session.last_rc_delta,
        )

        action: list[float] = []
        latency_ms = 0.0
        timeout_discard = False
        error_text: str | None = None
        candidate_rc = baseline_rc
        cmd_log = CommandLog(candidate=baseline_rc, safe=baseline_rc)

        try:
            if session.request.rc_control_mode != "baseline-only":
                infer_started_at = time.perf_counter()
                action = session.sdk.infer(obs, feedback)
                latency_ms = (time.perf_counter() - infer_started_at) * 1000.0
                session.action_summary.update(action, latency_ms)

                if (
                    not session.shield.should_accept_action(latency_ms, state_age_ms)
                    or not state_fresh
                    or not gazebo_fresh
                ):
                    timeout_discard = True
                    candidate_rc = baseline_rc
                else:
                    candidate_rc = _candidate_rc_from_action(session, baseline_rc, copter_state, action)
        except RuntimeError as exc:
            error_text = str(exc)
            candidate_rc = baseline_rc

        safe_rc, shield_info = session.shield.filter_rc(candidate_rc)
        if error_text is not None or timeout_discard:
            shield_info.safety_triggered = True
        session.action_summary.update_shield(
            clipped=len(shield_info.clipped_fields) > 0,
            filtered=shield_info.filtered,
            safety=shield_info.safety_triggered,
        )

        if allow_send and session.writes_enabled:
            cmd_log = session.adapter.send_rc_cmd(safe_rc)
        else:
            cmd_log = CommandLog(candidate=candidate_rc, safe=safe_rc, sent=False)
        if cmd_log.sent:
            session.action_summary.update_sent()

        session.last_shield_info = shield_info
        session.last_rc_delta = _rc_delta_from_neutral(
            session.action_adapter.baseline_rc_neutral(),
            safe_rc,
        )

        return {
            "obs": obs,
            "feedback": feedback,
            "feedback_diagnostics": session.feedback_builder.last_diagnostics.to_log_dict(),
            "action": action,
            "latency_ms": latency_ms,
            "baseline_rc": baseline_rc,
            "candidate_rc": candidate_rc,
            "safe_rc": safe_rc,
            "rc_delta": list(session.last_rc_delta),
            "shield_info": shield_info,
            "timeout_discard": timeout_discard,
            "cmd_log": cmd_log,
            "error": error_text or cmd_log.error,
        }

    def _run_step_with_session(
        self,
        session: _ControlSession,
        copter_state: CopterState,
        requested_send: bool,
    ) -> StepControlResponse:
        with session.frame_lock:
            result = self._infer_and_send_rc(
                session=session,
                copter_state=copter_state,
                state_age_ms=0.0,
                state_fresh=True,
                gazebo_fresh=True,
                allow_send=requested_send,
            )
        return _step_response_from_result(result)

    def _run_step_with_temporary_context(
        self,
        request: StepControlRequest,
        copter_state: CopterState,
    ) -> StepControlResponse:
        config = load_config(request.config, use_real=request.use_real_limits)
        sdk = make_inference_client(
            sdk_mode=request.sdk_mode,
            host=request.host,
            port=request.port,
            timeout_s=request.timeout_s,
            username=request.username,
            password=request.password,
            resume=request.resume,
        )
        adapter = MavlinkAdapter(vehicle_type="copter", mode="dry-run", control="rc")
        session = _ControlSession(
            request=_start_request_from_step(request),
            config=config,
            log_file=Path("/tmp/flight_bridge_api_step.jsonl"),
            adapter=adapter,
            sdk=sdk,
            logger=JsonlLogger(Path("/tmp/flight_bridge_api_step.jsonl")),
        )
        session.load_optional_adapter()

        with sdk:
            result = self._infer_and_send_rc(
                session=session,
                copter_state=copter_state,
                state_age_ms=0.0,
                state_fresh=True,
                gazebo_fresh=True,
                allow_send=False,
            )
        return _step_response_from_result(result)

    def _latest_gazebo_odom(self, session: _ControlSession) -> GazeboOdom | None:
        if session.gazebo_reader is None:
            return None
        return session.gazebo_reader.latest(timeout=0.0)

    def _build_log_record(
        self,
        session: _ControlSession,
        frame_index: int,
        raw_state: dict[str, Any],
        vehicle_mode: str,
        armed: bool,
        copter_state: CopterState,
        result: dict[str, Any],
        state_age_ms: float,
        gazebo_age_ms: float | None,
        gazebo_odom: GazeboOdom | None,
    ) -> dict[str, Any]:
        cmd_log: CommandLog = result["cmd_log"]
        shield_info: ShieldInfo = result["shield_info"]
        return {
            "ts": time.time(),
            "frame": frame_index,
            "phase": 1,
            "api": True,
            "mode": session.request.mode,
            "control": "rc",
            "rc_control_mode": session.request.rc_control_mode,
            "vehicle_mode": vehicle_mode,
            "armed": armed,
            "state": raw_state,
            "gazebo_odom": (
                gazebo_odom.to_log_dict(session.gazebo_home)
                if gazebo_odom is not None else None
            ),
            "copter_state": _copter_state_to_log(copter_state),
            "obs": result["obs"],
            "feedback": result["feedback"],
            "feedback_diagnostics": result["feedback_diagnostics"],
            "action": result["action"],
            "latency_ms": result["latency_ms"],
            "state_age_ms": state_age_ms,
            "gazebo_age_ms": gazebo_age_ms,
            "timeout_discard": result["timeout_discard"],
            "baseline_rc": result["baseline_rc"].to_list(),
            "candidate_command": result["candidate_rc"].to_list(),
            "safe_command": result["safe_rc"].to_list(),
            "rc_delta": result["rc_delta"],
            "gazebo_reset": None,
            "reset_recovery": None,
            "shield": _shield_info_to_dict(shield_info),
            "sent": cmd_log.sent,
            "cmd_method": cmd_log.method,
            "error": result["error"],
        }


def _candidate_rc_from_action(
    session: _ControlSession,
    baseline_rc: RcCommand,
    copter_state: CopterState,
    action: list[float],
) -> RcCommand:
    if session.request.rc_control_mode == "sdk-direct":
        sdk_rc = session.action_adapter.to_rc_command(action)
        sdk_rc = _protect_low_altitude_throttle(
            sdk_rc,
            baseline_rc,
            copter_state,
            target_alt_m=session.request.target_alt_m,
        )
        return _protect_low_altitude_attitude(
            sdk_rc,
            copter_state,
            level_until_alt_m=session.request.sdk_direct_level_until_alt_m,
        )
    if session.action_adapter.has_learned:
        return session.action_adapter.gaia_action_to_rc(baseline_rc, action)
    sdk_rc = session.action_adapter.to_rc_command(action)
    return session.action_adapter.residual_blend_rc(baseline_rc, sdk_rc)


def _start_request_from_step(request: StepControlRequest) -> ControlStartRequest:
    return ControlStartRequest(
        mode="dry-run",
        sdk_mode=request.sdk_mode,
        host=request.host,
        port=request.port,
        timeout_s=request.timeout_s,
        username=request.username,
        password=request.password,
        resume=request.resume,
        config=request.config,
        use_real_limits=request.use_real_limits,
        rc_control_mode=request.rc_control_mode,
        adapter_mode=request.adapter_mode,
        gaia_adapter_config=request.gaia_adapter_config,
    )


def _session_info(session: _ControlSession) -> dict[str, Any]:
    return {
        "mode": session.request.mode,
        "control": "rc",
        "sdk_mode": session.request.sdk_mode,
        "rate_hz": session.request.rate_hz,
        "log_file": str(session.log_file),
        "summary": session.action_summary.format_line(),
    }


def _status_value(
    session: _ControlSession | None,
    session_info: dict[str, Any] | None,
    key: str,
) -> Any:
    if session is not None:
        if key == "mode":
            return session.request.mode
        if key == "control":
            return "rc"
        if key == "sdk_mode":
            return session.request.sdk_mode
        if key == "rate_hz":
            return session.request.rate_hz
        if key == "log_file":
            return str(session.log_file)
    if session_info is None:
        return None
    return session_info.get(key)


def _step_response_from_result(result: dict[str, Any]) -> StepControlResponse:
    candidate = result["candidate_rc"]
    safe = result["safe_rc"]
    cmd_log: CommandLog = result["cmd_log"]
    return StepControlResponse(
        obs=result["obs"],
        feedback=result["feedback"],
        feedback_diagnostics=result["feedback_diagnostics"],
        action=result["action"],
        latency_ms=result["latency_ms"],
        baseline_command=result["baseline_rc"].to_list(),
        candidate_command=candidate.to_list(),
        candidate_command_struct=_rc_model(candidate),
        safe_command=safe.to_list(),
        safe_command_struct=_rc_model(safe),
        shield=_shield_model(result["shield_info"]),
        sent=cmd_log.sent,
        cmd_method=cmd_log.method,
        error=result["error"],
    )


def _rc_delta_from_neutral(neutral: RcCommand, safe: RcCommand) -> list[float]:
    return [
        (safe.roll - neutral.roll) / 500.0,
        (safe.pitch - neutral.pitch) / 500.0,
        (safe.throttle - neutral.throttle) / 500.0,
        (safe.yaw - neutral.yaw) / 500.0,
    ]


def _rc_model(cmd: RcCommand) -> RcCommandModel:
    return RcCommandModel(
        roll_pwm=cmd.roll,
        pitch_pwm=cmd.pitch,
        throttle_pwm=cmd.throttle,
        yaw_pwm=cmd.yaw,
    )


def _rc_model_from_list(values: Any) -> RcCommandModel | None:
    if not isinstance(values, list) or len(values) != 4:
        return None
    return RcCommandModel(
        roll_pwm=int(values[0]),
        pitch_pwm=int(values[1]),
        throttle_pwm=int(values[2]),
        yaw_pwm=int(values[3]),
    )


def _shield_model(info: ShieldInfo) -> ShieldInfoModel:
    return ShieldInfoModel(**_shield_info_to_dict(info))


def _shield_info_to_dict(info: ShieldInfo) -> dict[str, Any]:
    return {
        "saturated": info.saturated,
        "safety_triggered": info.safety_triggered,
        "clipped_fields": list(info.clipped_fields),
        "filtered": info.filtered,
        "in_deadband": info.in_deadband,
    }


def _gazebo_age_ms(odom: GazeboOdom | None, now: float) -> float | None:
    if odom is None or odom.updated_at <= 0.0:
        return None
    return (now - odom.updated_at) * 1000.0


service = FlightControlService()
app = FastAPI(title="Flight Bridge Control API", version="0.1.0")


@app.get("/health")
def health() -> dict[str, Any]:
    current = service.status()
    return {
        "ok": True,
        "service": "flight_bridge",
        "running": current.running,
        "frame": current.frame,
        "latest_error": current.latest_error,
    }


@app.post(
    "/control/start",
    response_model=ControlStartResponse,
    status_code=status.HTTP_202_ACCEPTED,
)
def start_control(request: ControlStartRequest) -> ControlStartResponse:
    try:
        return service.start(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail=str(exc)) from exc
    except ConnectionError as exc:
        raise HTTPException(status_code=status.HTTP_502_BAD_GATEWAY, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc


@app.post("/control/stop", response_model=ControlStopResponse)
def stop_control() -> ControlStopResponse:
    return service.stop()


@app.get("/control/status", response_model=ControlStatusResponse)
def control_status() -> ControlStatusResponse:
    return service.status()


@app.post("/control/step", response_model=StepControlResponse)
def control_step(request: StepControlRequest) -> StepControlResponse:
    try:
        return service.step(request)
    except Exception as exc:
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail=str(exc)) from exc
