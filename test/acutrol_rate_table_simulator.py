#!/usr/bin/env python3
"""ACUTROL rate-table TCP/IP simulator for Raspberry Pi.

This server emulates the subset of ACL commands used by the gyroscope test
software turntable HAL, so communication can be validated before using the
real rate table.

Supported commands (short and long forms):
- :M:R <axis> / :MODE:RATE <axis>
- :M:P <axis> / :MODE:POSITION <axis>
- :M:O <axis> / :MODE:OFF <axis>
- :D:R <axis>,<rate> / :DEMAND:RATE <axis>,<rate>
- :D:P <axis>,<pos> / :DEMAND:POSITION <axis>,<pos>
- :R:R <axis> / :READ:RATE <axis>
- :R:P <axis> / :READ:POSITION <axis>
- *IDN?

Response format:
- Queries return plain ASCII value + LF, e.g. "12.340000\n"
- Commands do not return data (simple instrument-like behavior)
"""

from __future__ import annotations

import argparse
import math
import signal
import socketserver
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple


@dataclass
class AxisState:
    mode: str = "OFF"
    rate_deg_s: float = 0.0
    pos_deg: float = 0.0
    last_update_s: float = 0.0


class AcutrolSimulatorState:
    def __init__(self) -> None:
        now = time.monotonic()
        self._axes: Dict[int, AxisState] = {
            1: AxisState(last_update_s=now),
            2: AxisState(last_update_s=now),
        }
        self._lock = threading.Lock()

    def handle_message(self, text: str) -> Optional[str]:
        """Handle one line that may include multiple ACL messages separated by ';'."""
        responses = []
        for chunk in text.split(";"):
            cmd = chunk.strip()
            if not cmd:
                continue
            response = self._handle_single(cmd)
            if response is not None:
                responses.append(response)

        if not responses:
            return None
        return ";".join(responses) + "\n"

    def _handle_single(self, command: str) -> Optional[str]:
        upper = command.upper()

        if upper == "*IDN?":
            return "ACUTRONIC,ACUTROL3000-SIM,0000000000,SIM-1.0"

        if not upper.startswith(":"):
            return None

        parts = upper.split(None, 1)
        head = parts[0]
        args = parts[1] if len(parts) > 1 else ""

        path = [p for p in head.split(":") if p]
        if len(path) != 2:
            return None

        command_group = self._normalize_main(path[0])
        sub = self._normalize_sub(path[1])

        if command_group == "MODE":
            axis = self._parse_axis(args)
            if axis is None:
                return None
            if sub in ("RATE", "POSITION", "OFF"):
                with self._lock:
                    self._advance_axis(axis)
                    self._axes[axis].mode = sub
                    if sub == "OFF":
                        self._axes[axis].rate_deg_s = 0.0
            return None

        if command_group == "DEMAND":
            parsed = self._parse_axis_value(args)
            if parsed is None:
                return None
            axis, value = parsed
            with self._lock:
                self._advance_axis(axis)
                if sub == "RATE":
                    self._axes[axis].rate_deg_s = value
                elif sub == "POSITION":
                    self._axes[axis].pos_deg = self._wrap_360(value)
            return None

        if command_group == "READ":
            axis = self._parse_axis(args)
            if axis is None:
                return None
            with self._lock:
                self._advance_axis(axis)
                if sub == "RATE":
                    return f"{self._axes[axis].rate_deg_s:.6f}"
                if sub == "POSITION":
                    return f"{self._axes[axis].pos_deg:.6f}"
            return None

        return None

    @staticmethod
    def _normalize_main(token: str) -> str:
        mapping = {
            "M": "MODE",
            "MODE": "MODE",
            "D": "DEMAND",
            "DEMAND": "DEMAND",
            "R": "READ",
            "READ": "READ",
        }
        return mapping.get(token, token)

    @staticmethod
    def _normalize_sub(token: str) -> str:
        mapping = {
            "R": "RATE",
            "RATE": "RATE",
            "P": "POSITION",
            "POS": "POSITION",
            "POSITION": "POSITION",
            "O": "OFF",
            "OFF": "OFF",
        }
        return mapping.get(token, token)

    def _parse_axis(self, args: str) -> Optional[int]:
        raw = args.strip().split(",", 1)[0].strip()
        if not raw:
            return None
        try:
            axis = int(raw)
        except ValueError:
            return None
        return axis if axis in self._axes else None

    def _parse_axis_value(self, args: str) -> Optional[Tuple[int, float]]:
        fields = [f.strip() for f in args.split(",")]
        if len(fields) < 2:
            return None
        try:
            axis = int(fields[0])
            value = float(fields[1])
        except ValueError:
            return None
        if axis not in self._axes:
            return None
        if math.isnan(value) or math.isinf(value):
            return None
        return axis, value

    def _advance_axis(self, axis: int) -> None:
        st = self._axes[axis]
        now = time.monotonic()
        dt = max(0.0, now - st.last_update_s)
        if st.mode == "RATE":
            st.pos_deg = self._wrap_360(st.pos_deg + st.rate_deg_s * dt)
        st.last_update_s = now

    @staticmethod
    def _wrap_360(value: float) -> float:
        wrapped = value % 360.0
        if wrapped < 0.0:
            wrapped += 360.0
        return wrapped


class _ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


class _RequestHandler(socketserver.BaseRequestHandler):
    state: AcutrolSimulatorState

    def handle(self) -> None:
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        print(f"[sim] client connected: {peer}")

        buffer = b""
        while True:
            try:
                chunk = self.request.recv(4096)
            except OSError:
                break

            if not chunk:
                break

            buffer += chunk

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                text = line.decode("ascii", errors="ignore").rstrip("\r")
                if not text:
                    continue

                response = self.state.handle_message(text)
                if response is not None:
                    try:
                        self.request.sendall(response.encode("ascii", errors="ignore"))
                    except OSError:
                        break

        print(f"[sim] client disconnected: {peer}")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ACUTROL rate-table TCP/IP simulator")
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Bind address (0.0.0.0 to accept from LAN)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=23,
        help="TCP port (default 23 to mimic ACUTROL; use sudo on Linux for <1024)",
    )
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    state = AcutrolSimulatorState()
    _RequestHandler.state = state

    with _ThreadingTCPServer((args.host, args.port), _RequestHandler) as server:
        print("[sim] ACUTROL simulator ready")
        print(f"[sim] listening on {args.host}:{args.port}")
        print("[sim] stop with Ctrl+C")

        def _signal_handler(signum, frame):  # type: ignore[no-untyped-def]
            del signum, frame
            # shutdown() must not be called from the same thread as serve_forever().
            threading.Thread(target=server.shutdown, daemon=True).start()

        signal.signal(signal.SIGINT, _signal_handler)
        signal.signal(signal.SIGTERM, _signal_handler)

        try:
            server.serve_forever(poll_interval=0.3)
        except KeyboardInterrupt:
            pass
        finally:
            server.server_close()

    print("[sim] simulator stopped")
    return 0

# sudo python3 acutrol_rate_table_simulator.py --host 192.168.53.1 --port 23

if __name__ == "__main__":
    raise SystemExit(main())
