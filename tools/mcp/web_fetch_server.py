#!/usr/bin/env python3
"""
Web Fetch MCP Server — fetches URLs directly from the local machine,
bypassing claude.ai domain verification.

Supports: HTML pages (→ readable text), PDFs (→ text via pdftotext).

Protocol: stdio JSON-RPC 2.0 (same convention as eagle_schematic_server.py)
"""

from __future__ import annotations

import html.parser
import io
import json
import os
import re
import subprocess
import sys
import tempfile
import urllib.request
from typing import Any
from urllib.parse import urlparse

# ── HTML → readable text ────────────────────────────────────────────────

class _HTMLStripper(html.parser.HTMLParser):
    """Strip tags from HTML, keep text, add minimal line-breaks."""
    def __init__(self) -> None:
        super().__init__()
        self.parts: list[str] = []
        self._skip_depth = 0
        self._skip_tags = {
            "script", "style", "nav", "footer", "head",
            "noscript", "iframe", "svg", "canvas",
        }
        self._block_tags = {
            "div", "p", "h1", "h2", "h3", "h4", "h5", "h6",
            "li", "tr", "section", "article", "header", "main",
            "aside", "figure", "figcaption", "pre", "blockquote",
            "table", "ul", "ol", "dl", "form", "fieldset",
        }

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag in self._skip_tags:
            self._skip_depth += 1
        if tag in ("br", "hr"):
            self.parts.append("\n")

    def handle_endtag(self, tag: str) -> None:
        if tag in self._skip_tags and self._skip_depth > 0:
            self._skip_depth -= 1
        if tag in self._block_tags:
            self.parts.append("\n")

    def handle_data(self, data: str) -> None:
        if self._skip_depth > 0:
            return
        text = data.strip()
        if text:
            self.parts.append(text + " ")

    def handle_entityref(self, name: str) -> None:
        if self._skip_depth > 0:
            return
        entity_map = {
            "amp": "&", "lt": "<", "gt": ">", "quot": '"',
            "apos": "'", "nbsp": " ",
        }
        self.parts.append(entity_map.get(name, f"&{name};"))


def html_to_text(html_bytes: bytes) -> str:
    """Convert HTML bytes to plain text."""
    stripper = _HTMLStripper()
    # Decode, replacing invalid bytes
    html_str = html_bytes.decode("utf-8", errors="replace")
    stripper.feed(html_str)
    raw = "".join(stripper.parts)
    # Collapse blank lines (max 2 consecutive newlines)
    raw = re.sub(r"\n{3,}", "\n\n", raw)
    # Collapse multiple spaces
    raw = re.sub(r"[ \t]{2,}", " ", raw)
    return raw.strip()


# ── PDF → text via pdftotext ────────────────────────────────────────────

def pdf_to_text(pdf_bytes: bytes) -> str:
    """Convert PDF bytes to text using system pdftotext."""
    with tempfile.NamedTemporaryFile(suffix=".pdf", delete=False) as tmp:
        tmp.write(pdf_bytes)
        tmp_path = tmp.name
    try:
        result = subprocess.run(
            ["pdftotext", "-layout", tmp_path, "-"],
            capture_output=True,
            text=True,
            timeout=60,
        )
        if result.returncode != 0:
            return f"[pdftotext error] {result.stderr}"
        return result.stdout.strip()
    finally:
        os.unlink(tmp_path)


# ── MIME / content-type sniffing ────────────────────────────────────────

def _sniff_type(response: urllib.request.addinfourl) -> str:
    """Guess content type from response headers or URL extension.
    Returns 'html', 'pdf', or 'other'."""
    ct = response.headers.get_content_type()  # e.g. 'text/html'
    if "pdf" in ct or "pdf" in response.headers.get("Content-Disposition", ""):
        return "pdf"
    if "html" in ct:
        return "html"
    return "other"


# ── URL fetching ────────────────────────────────────────────────────────

_USER_AGENT = (
    "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"
)

def fetch_url(url: str, max_bytes: int = 5_000_000) -> dict[str, Any]:
    """Fetch a URL and return its text content.
    Handles HTML (→ plain text) and PDF (→ pdftotext)."""
    parsed = urlparse(url)
    if parsed.scheme not in ("http", "https"):
        return {"error": f"Unsupported scheme: {parsed.scheme}"}

    req = urllib.request.Request(url, headers={"User-Agent": _USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            content_type = resp.headers.get_content_type()
            raw = resp.read(max_bytes)
    except urllib.error.HTTPError as e:
        return {"error": f"HTTP {e.code}: {e.reason}", "url": url}
    except urllib.error.URLError as e:
        return {"error": f"URL error: {e.reason}", "url": url}
    except Exception as e:
        return {"error": str(e), "url": url}

    kind = _sniff_type(resp)
    text: str
    if kind == "pdf" or url.lower().endswith(".pdf"):
        text = pdf_to_text(raw)
    elif kind == "html" or kind == "other":
        # Try HTML parsing first; if it yields nothing useful, treat as plain text
        text = html_to_text(raw)
        if len(text) < 100 and len(raw) > 100:
            # Probably not actually HTML — return raw with a note
            text = raw.decode("utf-8", errors="replace")[:10_000]
    else:
        text = raw.decode("utf-8", errors="replace")[:10_000]

    # Truncate to avoid flooding context
    if len(text) > 50_000:
        text = text[:50_000] + "\n\n[... truncated at 50000 chars ...]"

    return {
        "url": url,
        "content_type": content_type,
        "content_length": len(raw),
        "text": text,
    }


# ── MCP JSON-RPC glue ──────────────────────────────────────────────────

def _text_response(payload: Any) -> dict[str, Any]:
    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(payload, ensure_ascii=False, indent=2),
            }
        ]
    }


def _tool_schema() -> list[dict[str, Any]]:
    return [
        {
            "name": "fetch",
            "description": (
                "Fetch a URL (HTTP/HTTPS) and return its content as text. "
                "Supports HTML pages (stripped to readable plain text) and "
                "PDF files (converted via pdftotext). Use this to read papers, "
                "documentation, or any web content."
            ),
            "inputSchema": {
                "type": "object",
                "required": ["url"],
                "properties": {
                    "url": {
                        "type": "string",
                        "description": "The URL to fetch (must start with http:// or https://)",
                    },
                    "max_bytes": {
                        "type": "integer",
                        "description": "Maximum bytes to download (default 5MB)",
                        "minimum": 1,
                        "maximum": 50_000_000,
                    },
                },
            },
        },
        {
            "name": "fetch_pdf",
            "description": (
                "Fetch a PDF URL and extract its text via pdftotext. "
                "Useful for reading academic papers from arxiv.org, etc."
            ),
            "inputSchema": {
                "type": "object",
                "required": ["url"],
                "properties": {
                    "url": {
                        "type": "string",
                        "description": "The PDF URL to fetch",
                    },
                },
            },
        },
    ]


def _handle_tool_call(name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    url = str(arguments.get("url", ""))
    if not url.startswith(("http://", "https://")):
        return _text_response({"error": "URL must start with http:// or https://", "url": url})

    if name == "fetch":
        max_bytes = int(arguments.get("max_bytes", 5_000_000))
        result = fetch_url(url, max_bytes=max_bytes)
        return _text_response(result)
    elif name == "fetch_pdf":
        result = fetch_url(url)
        return _text_response(result)
    else:
        raise ValueError(f"Unknown tool: {name}")


def _send_response(message_id: Any, result: Any = None, error: Any = None) -> None:
    response: dict[str, Any] = {"jsonrpc": "2.0", "id": message_id}
    if error is None:
        response["result"] = result
    else:
        response["error"] = error
    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def run_server() -> None:
    for raw_line in sys.stdin:
        if not raw_line.strip():
            continue
        try:
            request = json.loads(raw_line)
            method = request.get("method")
            message_id = request.get("id")

            if method == "initialize":
                _send_response(
                    message_id,
                    {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "web-fetch", "version": "0.1.0"},
                    },
                )
            elif method == "notifications/initialized":
                continue
            elif method == "tools/list":
                _send_response(message_id, {"tools": _tool_schema()})
            elif method == "tools/call":
                params = request.get("params", {})
                result = _handle_tool_call(
                    name=params.get("name", ""),
                    arguments=params.get("arguments", {}) or {},
                )
                _send_response(message_id, result)
            else:
                _send_response(
                    message_id,
                    error={"code": -32601, "message": f"Method not found: {method}"},
                )
        except Exception as exc:
            _send_response(
                request.get("id") if "request" in locals() else None,
                error={"code": -32000, "message": str(exc)},
            )


if __name__ == "__main__":
    run_server()
