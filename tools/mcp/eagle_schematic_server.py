#!/usr/bin/env python3
"""Small stdio MCP server for inspecting Eagle schematic XML files."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Part:
    name: str
    value: str
    deviceset: str
    library: str


@dataclass(frozen=True)
class NetConnection:
    part: str
    gate: str
    pin: str


class EagleSchematic:
    def __init__(self, schematic_path: Path) -> None:
        if not schematic_path.exists():
            raise FileNotFoundError(f"Schematic does not exist: {schematic_path}")
        self.schematic_path = schematic_path
        self.root = ET.parse(schematic_path).getroot()
        self.parts = self._read_parts()
        self.nets = self._read_nets()

    def _read_parts(self) -> dict[str, Part]:
        parts: dict[str, Part] = {}
        for part_node in self.root.findall(".//parts/part"):
            name = part_node.get("name", "")
            if not name:
                continue
            parts[name] = Part(
                name=name,
                value=part_node.get("value", ""),
                deviceset=part_node.get("deviceset", ""),
                library=part_node.get("library", ""),
            )
        return parts

    def _read_nets(self) -> dict[str, list[NetConnection]]:
        nets: dict[str, list[NetConnection]] = {}
        for net_node in self.root.findall(".//sheets/sheet/nets/net"):
            net_name = net_node.get("name", "")
            if not net_name:
                continue
            connections = nets.setdefault(net_name, [])
            for pinref in net_node.findall(".//pinref"):
                connections.append(
                    NetConnection(
                        part=pinref.get("part", ""),
                        gate=pinref.get("gate", ""),
                        pin=pinref.get("pin", ""),
                    )
                )
        return nets

    def summary(self) -> dict[str, Any]:
        useful_text = []
        for text_node in self.root.findall(".//sheets/sheet/plain/text"):
            text = "".join(text_node.itertext()).strip()
            if text and not text.startswith(">"):
                useful_text.append(text)
        return {
            "file": str(self.schematic_path),
            "eagle_version": self.root.get("version", ""),
            "part_count": len(self.parts),
            "net_count": len(self.nets),
            "labels": sorted(set(useful_text))[:80],
        }

    def list_parts(self, query: str = "", limit: int = 80) -> list[dict[str, str]]:
        query_lower = query.lower()
        matches = []
        for part in sorted(self.parts.values(), key=lambda item: item.name):
            haystack = f"{part.name} {part.value} {part.deviceset} {part.library}".lower()
            if query_lower and query_lower not in haystack:
                continue
            matches.append(part.__dict__)
            if len(matches) >= limit:
                break
        return matches

    def list_nets(self, query: str = "", limit: int = 120) -> list[dict[str, Any]]:
        query_lower = query.lower()
        matches = []
        for net_name in sorted(self.nets):
            if query_lower and query_lower not in net_name.lower():
                continue
            matches.append({"name": net_name, "connection_count": len(self.nets[net_name])})
            if len(matches) >= limit:
                break
        return matches

    def net_connections(self, net_name: str) -> dict[str, Any]:
        matches = [
            name for name in self.nets
            if name.lower() == net_name.lower()
        ]
        if not matches:
            return {"net": net_name, "connections": [], "error": "net not found"}
        canonical_name = matches[0]
        return {
            "net": canonical_name,
            "connections": [connection.__dict__ for connection in self.nets[canonical_name]],
        }

    def part_connections(self, part_name: str) -> dict[str, Any]:
        matches = [
            name for name in self.parts
            if name.lower() == part_name.lower()
        ]
        if not matches:
            return {"part": part_name, "connections": [], "error": "part not found"}
        canonical_name = matches[0]
        connections = []
        for net_name, net_connections in sorted(self.nets.items()):
            for connection in net_connections:
                if connection.part == canonical_name:
                    connections.append(
                        {"net": net_name, "gate": connection.gate, "pin": connection.pin}
                    )
        return {
            "part": self.parts[canonical_name].__dict__,
            "connections": connections,
        }


def make_text_response(payload: Any) -> dict[str, Any]:
    return {
        "content": [
            {
                "type": "text",
                "text": json.dumps(payload, ensure_ascii=False, indent=2),
            }
        ]
    }


def tool_schema() -> list[dict[str, Any]]:
    return [
        {
            "name": "summary",
            "description": "Summarize the loaded Eagle schematic.",
            "inputSchema": {"type": "object", "properties": {}},
        },
        {
            "name": "list_parts",
            "description": "List schematic parts, optionally filtered by text.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "query": {"type": "string"},
                    "limit": {"type": "integer", "minimum": 1, "maximum": 300},
                },
            },
        },
        {
            "name": "list_nets",
            "description": "List net names, optionally filtered by text.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "query": {"type": "string"},
                    "limit": {"type": "integer", "minimum": 1, "maximum": 300},
                },
            },
        },
        {
            "name": "net_connections",
            "description": "Show all part pins connected to a named net.",
            "inputSchema": {
                "type": "object",
                "required": ["net_name"],
                "properties": {"net_name": {"type": "string"}},
            },
        },
        {
            "name": "part_connections",
            "description": "Show all nets connected to a named part.",
            "inputSchema": {
                "type": "object",
                "required": ["part_name"],
                "properties": {"part_name": {"type": "string"}},
            },
        },
    ]


def handle_tool_call(schematic: EagleSchematic, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    if name == "summary":
        return make_text_response(schematic.summary())
    if name == "list_parts":
        return make_text_response(
            schematic.list_parts(
                query=str(arguments.get("query", "")),
                limit=int(arguments.get("limit", 80)),
            )
        )
    if name == "list_nets":
        return make_text_response(
            schematic.list_nets(
                query=str(arguments.get("query", "")),
                limit=int(arguments.get("limit", 120)),
            )
        )
    if name == "net_connections":
        return make_text_response(schematic.net_connections(str(arguments["net_name"])))
    if name == "part_connections":
        return make_text_response(schematic.part_connections(str(arguments["part_name"])))
    raise ValueError(f"Unknown tool: {name}")


def send_response(message_id: Any, result: Any = None, error: Any = None) -> None:
    response: dict[str, Any] = {"jsonrpc": "2.0", "id": message_id}
    if error is None:
        response["result"] = result
    else:
        response["error"] = error
    sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
    sys.stdout.flush()


def run_server(schematic: EagleSchematic) -> None:
    for raw_line in sys.stdin:
        if not raw_line.strip():
            continue
        try:
            request = json.loads(raw_line)
            method = request.get("method")
            message_id = request.get("id")

            if method == "initialize":
                send_response(
                    message_id,
                    {
                        "protocolVersion": "2024-11-05",
                        "capabilities": {"tools": {}},
                        "serverInfo": {"name": "eagle-schematic", "version": "0.1.0"},
                    },
                )
            elif method == "notifications/initialized":
                continue
            elif method == "tools/list":
                send_response(message_id, {"tools": tool_schema()})
            elif method == "tools/call":
                params = request.get("params", {})
                result = handle_tool_call(
                    schematic,
                    name=params.get("name", ""),
                    arguments=params.get("arguments", {}) or {},
                )
                send_response(message_id, result)
            else:
                send_response(
                    message_id,
                    error={"code": -32601, "message": f"Method not found: {method}"},
                )
        except Exception as exc:
            send_response(
                request.get("id") if "request" in locals() else None,
                error={"code": -32000, "message": str(exc)},
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("schematic", type=Path, help="Path to an Eagle .sch XML file")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_server(EagleSchematic(args.schematic.resolve()))


if __name__ == "__main__":
    main()
