"""
websocket_client.py — Async generator that streams TelemetrySamples from the
WebSocket server run by serial_bridge.py.

Expected telemetry line format (produced by the STM32 firmware):
    $T,{timestamp_ms},{pos_deg×10},{vel_rads×10},{acc_rads2×10},{co×10}\r\n

Field encoding mirrors the READ Modbus registers (×10, int16 on wire):
    pos_deg×10    → divide by 10  → degrees
    vel_rads×10   → divide by 10  → rad/s
    co×10         → divide by 10  → normalised CO

The parser is deliberately lenient: malformed lines are silently skipped
so a single corrupt packet never stalls the stream.
"""

from __future__ import annotations

import asyncio
import logging
from typing import AsyncIterator, Optional

import websockets
import websockets.exceptions

from .relay_analyzer import TelemetrySample

log = logging.getLogger(__name__)

_EXPECTED_FIELD_COUNT = 6   # $T, ts, pos, vel, acc, co
_RECONNECT_DELAY_S    = 1.0


def _parse(line: str) -> Optional[TelemetrySample]:
    """Return a TelemetrySample or None if the line cannot be decoded."""
    line = line.strip()
    if not line.startswith("$"):
        return None
    try:
        parts = line[1:].split(",")   # strip leading '$'
        if len(parts) < _EXPECTED_FIELD_COUNT:
            return None
        # parts[0] is the type tag ("T")
        timestamp_ms   = float(parts[1])
        position_deg   = int(parts[2])  / 10.0
        velocity_rads  = int(parts[3])  / 10.0
        # parts[4] = acceleration — not required by the analyser but kept for future
        control_output = int(parts[5])  / 10.0
        return TelemetrySample(
            timestamp_ms=timestamp_ms,
            position_deg=position_deg,
            velocity_rads=velocity_rads,
            control_output=control_output,
        )
    except (ValueError, IndexError):
        log.debug("Telemetry parse failed: %r", line)
        return None


class WebSocketTelemetryClient:
    """
    Async generator that yields `TelemetrySample` objects indefinitely.

    Re-connects automatically after any disconnect or network error.
    Call `stop()` to signal the generator to exit cleanly at the next
    reconnect boundary.
    """

    def __init__(self, url: str = "ws://localhost:8765") -> None:
        self.url = url
        self._stop = asyncio.Event()

    def stop(self) -> None:
        self._stop.set()

    async def stream(self) -> AsyncIterator[TelemetrySample]:
        while not self._stop.is_set():
            try:
                async with websockets.connect(
                    self.url,
                    ping_interval=20,
                    ping_timeout=10,
                    open_timeout=5,
                ) as ws:
                    log.info("WS connected: %s", self.url)
                    async for message in ws:
                        if self._stop.is_set():
                            return
                        # A single WS message may contain multiple telemetry lines
                        # (the bridge batches lines every 5 ms).
                        for line in message.splitlines():
                            sample = _parse(line)
                            if sample is not None:
                                yield sample

            except (websockets.exceptions.ConnectionClosed, OSError) as exc:
                if self._stop.is_set():
                    return
                log.warning("WS disconnected (%s) — retry in %.1f s", exc, _RECONNECT_DELAY_S)
                await asyncio.sleep(_RECONNECT_DELAY_S)
            except Exception as exc:
                if self._stop.is_set():
                    return
                log.error("WS unexpected error: %s", exc, exc_info=True)
                await asyncio.sleep(_RECONNECT_DELAY_S)
