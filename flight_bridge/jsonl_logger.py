"""Small JSONL logger for bridge telemetry."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class JsonlLogger:
    """Append one JSON object per line and flush every frame."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = None

    def __enter__(self) -> "JsonlLogger":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8")
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None

    def write(self, record: dict[str, Any]) -> None:
        if self._file is None:
            raise RuntimeError("JsonlLogger is not open")

        line = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        self._file.write(line + "\n")
        self._file.flush()

