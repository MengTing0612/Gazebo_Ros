"""Non-blocking SDK inference worker used by the ROS 2 control timer."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

from flight_bridge.sdk_client import InferenceClient, make_inference_client


@dataclass(frozen=True)
class SdkRequest:
    """One normalized observation and feedback pair for the inference service."""

    observation: list[float]
    feedback: list[float]


@dataclass(frozen=True)
class SdkResult:
    """Latest SDK output with its local receive time."""

    action: list[float]
    received_monotonic_s: float
    error: str | None = None


class SdkWorker:
    """Keep network inference off the deterministic ROS 2 control timer."""

    def __init__(
        self,
        mode: str,
        host: str,
        port: int,
        timeout_s: float,
        username: str,
        password: str,
        resume: bool,
    ) -> None:
        self._mode = mode
        self._connection = (host, port, timeout_s, username, password, resume)
        self._request_queue: queue.Queue[SdkRequest] = queue.Queue(maxsize=1)
        self._result_lock = threading.Lock()
        self._latest = SdkResult(action=[0.0] * 4, received_monotonic_s=0.0)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Start inference if SDK use was enabled."""
        if self._mode == "disabled" or self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="sdk-inference", daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stop the worker without blocking shutdown on a failed network peer."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.25)
            self._thread = None

    def submit(self, observation: list[float], feedback: list[float]) -> None:
        """Replace queued work so an overloaded SDK cannot accumulate latency."""
        if self._mode == "disabled":
            return
        request = SdkRequest(observation=list(observation), feedback=list(feedback))
        try:
            self._request_queue.put_nowait(request)
        except queue.Full:
            try:
                self._request_queue.get_nowait()
            except queue.Empty:
                pass
            self._request_queue.put_nowait(request)

    def latest(self) -> SdkResult:
        """Return the last action without waiting for I/O."""
        with self._result_lock:
            return self._latest

    def _run(self) -> None:
        host, port, timeout_s, username, password, resume = self._connection
        while not self._stop.is_set():
            try:
                with self._open_client(host, port, timeout_s, username, password, resume) as client:
                    self._serve_client(client)
            except Exception as exc:
                self._set_result([], f"SDK connection/inference failed: {exc}")
                self._stop.wait(timeout=1.0)

    def _serve_client(self, client: InferenceClient) -> None:
        while not self._stop.is_set():
            try:
                request = self._request_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._set_result(client.infer(request.observation, request.feedback), None)
            except Exception as exc:
                self._set_result([], f"SDK inference failed: {exc}")
                return

    def _open_client(
        self,
        host: str,
        port: int,
        timeout_s: float,
        username: str,
        password: str,
        resume: bool,
    ) -> InferenceClient:
        return make_inference_client(
            self._mode,
            host,
            port,
            timeout_s,
            username,
            password,
            resume,
        )

    def _set_result(self, action: list[float], error: str | None) -> None:
        with self._result_lock:
            self._latest = SdkResult(
                action=list(action),
                received_monotonic_s=time.monotonic(),
                error=error,
            )
