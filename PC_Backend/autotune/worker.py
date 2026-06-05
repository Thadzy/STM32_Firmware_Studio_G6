"""
worker.py — asyncio/Qt boundary layer.

AsyncWorker runs a private asyncio event loop inside a QThread.  All
communication with the Qt main thread goes through pyqtSignal / pyqtSlot,
which are thread-safe.  Callers on the Qt thread submit coroutines via
`asyncio.run_coroutine_threadsafe`.

Architecture
────────────
  Qt main thread
      ↕  signals / slots (thread-safe)
  AsyncWorker (QThread)
      └─ asyncio event loop
            ├─ WebSocketTelemetryClient.stream()   [ingest loop]
            └─ ModbusClient                        [command/response]

Design choice: the relay switching itself runs on the STM32 firmware
(activated by AT_CMD=StartRelay).  This worker is a pure observer on the
telemetry side; it only writes Modbus registers to start/stop/apply gains.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Optional

from PyQt5.QtCore import QObject, QThread, pyqtSignal

from .relay_analyzer import RelayAnalyzer, RelayState, TelemetrySample
from .modbus_client import AutoTuneModbusClient, LOOP_POSITION
from .websocket_client import WebSocketTelemetryClient

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Signal container — must be a QObject so pyqtSignal works
# ---------------------------------------------------------------------------

class _Signals(QObject):
    # (time_s, position_deg, velocity_rads, control_output)
    telemetry        = pyqtSignal(float, float, float, float)
    # RelayState integer
    analyzer_state   = pyqtSignal(int)
    # (Kp, Ki, Kd, Ku, Pu_s, amplitude_eu, cycle_count)
    gains_ready      = pyqtSignal(float, float, float, float, float, float, int)
    # (success, message)
    modbus_status    = pyqtSignal(bool, str)
    # free-text error
    error            = pyqtSignal(str)


# ---------------------------------------------------------------------------
# Worker thread
# ---------------------------------------------------------------------------

class AsyncWorker(QThread):
    """
    Owns the asyncio event loop and all I/O resources.

    Usage
    -----
    worker = AsyncWorker(ws_url=..., modbus_port=..., modbus_baudrate=...)
    worker.signals.telemetry.connect(my_slot)
    worker.start()
    # later:
    worker.start_autotune(...)
    worker.stop(); worker.wait()
    """

    def __init__(
        self,
        ws_url: str = "ws://localhost:8765",
        modbus_port: str = "COM12",
        modbus_baudrate: int = 230400,
        parent: Optional[QObject] = None,
    ) -> None:
        super().__init__(parent)
        self.signals = _Signals()

        self._ws_url          = ws_url
        self._modbus_port     = modbus_port
        self._modbus_baudrate = modbus_baudrate

        self._loop:       Optional[asyncio.AbstractEventLoop] = None
        self._loop_ready  = threading.Event()

        self._ws_client:  Optional[WebSocketTelemetryClient] = None
        self._modbus:     Optional[AutoTuneModbusClient]     = None
        self._analyzer:   Optional[RelayAnalyzer]            = None

        self._autotune_active = False

    # ------------------------------------------------------------------
    # QThread entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop_ready.set()
        try:
            self._loop.run_until_complete(self._main())
        except Exception as exc:
            log.exception("AsyncWorker loop crashed: %s", exc)
            self.signals.error.emit(f"Worker crash: {exc}")
        finally:
            pending = asyncio.all_tasks(self._loop)
            for t in pending:
                t.cancel()
            if pending:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            self._loop.close()

    # ------------------------------------------------------------------
    # Public API  (safe to call from the Qt main thread)
    # ------------------------------------------------------------------

    def stop(self) -> None:
        if self._ws_client is not None:
            self._ws_client.stop()
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._loop.stop)

    def start_autotune(
        self,
        relay_amplitude: float,
        setpoint_deg: float,
        loop_target: int,
        hysteresis: float,
        min_cycles: int,
        settle_time_s: float,
    ) -> None:
        self._schedule(self._cmd_start_autotune(
            relay_amplitude, setpoint_deg, loop_target,
            hysteresis, min_cycles, settle_time_s,
        ))

    def abort_autotune(self) -> None:
        self._schedule(self._cmd_abort())

    def apply_gains(self, kp: float, ki: float, kd: float) -> None:
        self._schedule(self._cmd_apply_gains(kp, ki, kd))

    # ------------------------------------------------------------------
    # Internal asyncio coroutines
    # ------------------------------------------------------------------

    async def _main(self) -> None:
        self._modbus = AutoTuneModbusClient(self._modbus_port, self._modbus_baudrate)
        connected = await self._modbus.connect()
        self.signals.modbus_status.emit(connected, "Modbus connected" if connected else "Modbus failed — check COM port")

        self._ws_client = WebSocketTelemetryClient(self._ws_url)
        async for sample in self._ws_client.stream():
            self.signals.telemetry.emit(
                sample.timestamp_ms / 1000.0,
                sample.position_deg,
                sample.velocity_rads,
                sample.control_output,
            )
            if self._autotune_active and self._analyzer is not None:
                await self._feed_analyzer(sample)

    async def _feed_analyzer(self, sample: TelemetrySample) -> None:
        self._analyzer.update(sample)
        self.signals.analyzer_state.emit(int(self._analyzer.state))

        if self._analyzer.state == RelayState.DONE:
            self._autotune_active = False
            # Tell STM32 to exit relay mode
            if self._modbus and self._modbus.connected:
                await self._modbus.abort()
            result = self._analyzer.result
            if result is not None:
                self.signals.gains_ready.emit(
                    result.Kp, result.Ki, result.Kd,
                    result.Ku, result.Pu_s,
                    result.oscillation_amplitude,
                    result.cycle_count,
                )

        elif self._analyzer.state == RelayState.FAULT:
            self._autotune_active = False
            if self._modbus and self._modbus.connected:
                await self._modbus.abort()
            self.signals.error.emit(
                "Auto-tune FAULT: no oscillation detected — check relay amplitude and setpoint."
            )

    async def _cmd_start_autotune(
        self,
        amplitude: float,
        setpoint_deg: float,
        loop_target: int,
        hysteresis: float,
        min_cycles: int,
        settle_time_s: float,
    ) -> None:
        self._analyzer = RelayAnalyzer(
            relay_amplitude=amplitude,
            setpoint=setpoint_deg,
            loop_target=loop_target,
            hysteresis=hysteresis,
            min_cycles=min_cycles,
            settle_time_s=settle_time_s,
        )
        self._analyzer.start()
        self._autotune_active = True
        self.signals.analyzer_state.emit(int(RelayState.SETTLING))

        if self._modbus and self._modbus.connected:
            ok = await self._modbus.start_relay(amplitude, setpoint_deg, hysteresis, loop_target)
            if not ok:
                self.signals.error.emit("Modbus: failed to send StartRelay command")

    async def _cmd_abort(self) -> None:
        self._autotune_active = False
        if self._analyzer is not None:
            self._analyzer.abort()
        if self._modbus and self._modbus.connected:
            await self._modbus.abort()
        self.signals.analyzer_state.emit(int(RelayState.IDLE))

    async def _cmd_apply_gains(self, kp: float, ki: float, kd: float) -> None:
        if self._modbus is None or not self._modbus.connected:
            self.signals.error.emit("Modbus not connected — cannot apply gains")
            return
        ok = await self._modbus.apply_gains(kp, ki, kd)
        self.signals.modbus_status.emit(ok, "Gains applied" if ok else "Apply gains FAILED")

    # ------------------------------------------------------------------
    # Thread-safe coroutine scheduling
    # ------------------------------------------------------------------

    def _schedule(self, coro) -> None:
        self._loop_ready.wait(timeout=5.0)
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(coro, self._loop)
