"""
relay_analyzer.py — Åström-Hägglund relay feedback analysis engine.

Pure math; zero Qt or I/O dependencies. The STM32 firmware runs the actual
relay switching (register AT_CMD=1 activates it).  This module is a passive
observer: it ingests the raw telemetry stream, detects oscillation peaks,
and synthesises ZN-PID gains.

Mathematics
-----------
  Relay amplitude     : d   (same value written to AT_RELAY_AMP)
  Oscillation amplitude: a   = median((max_i - min_i) / 2)
  Ultimate gain        : Ku  = 4d / (π·a)                [Åström 1984]
  Ultimate period      : Pu  = median(Δt between successive maxima)

Ziegler-Nichols closed-loop PID rules:
  Kp = 0.60 · Ku
  Ki = Kp / (0.50 · Pu)  =  1.20 · Ku / Pu
  Kd = Kp · (0.125 · Pu) =  0.075 · Ku · Pu
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------

@dataclass
class TelemetrySample:
    """One decoded telemetry frame from the WebSocket stream."""
    timestamp_ms: float    # absolute wall-clock time (ms) from firmware tick
    position_deg: float    # outer-loop PV  (degrees)
    velocity_rads: float   # inner-loop PV  (rad/s)
    control_output: float  # CO as written to the plant (normalised PWM or rad/s SP)


@dataclass
class ZNGains:
    """Computed PID gains plus the intermediate identification results."""
    Kp: float
    Ki: float
    Kd: float
    Ku: float              # ultimate gain
    Pu_s: float            # ultimate period in seconds
    oscillation_amplitude: float   # half peak-to-peak, in PV engineering units
    cycle_count: int       # number of complete oscillation cycles used


class RelayState(IntEnum):
    IDLE     = 0
    SETTLING = 1   # waiting for PV to reach the setpoint neighbourhood
    ACTIVE   = 2   # relay is running; collecting peaks
    DONE     = 3   # min_cycles reached; result available
    FAULT    = 4   # something went wrong (no oscillation, bad data, …)


# ---------------------------------------------------------------------------
# Core analyser
# ---------------------------------------------------------------------------

class RelayAnalyzer:
    """
    Passive observer for a relay feedback experiment.

    Call `start()` once, then feed every incoming `TelemetrySample` to
    `update()`.  When `state == DONE`, read `result` for the ZN gains.

    Parameters
    ----------
    relay_amplitude  : d in the Åström formula; must match AT_RELAY_AMP
    setpoint         : operating point around which the relay oscillates
    loop_target      : 0 = velocity inner loop, 1 = position outer loop
    hysteresis       : relay dead-band half-width (engineering units of PV)
    min_cycles       : minimum complete oscillation cycles before declaring DONE
    settle_time_s    : seconds to wait after `start()` before analysing peaks
    min_peak_spacing_s : guard against double-detection of the same peak
    """

    LOOP_VELOCITY = 0
    LOOP_POSITION = 1

    def __init__(
        self,
        relay_amplitude: float,
        setpoint: float,
        loop_target: int = LOOP_POSITION,
        hysteresis: float = 0.5,
        min_cycles: int = 4,
        settle_time_s: float = 2.0,
        min_peak_spacing_s: float = 0.05,
    ) -> None:
        self.d = relay_amplitude
        self.setpoint = setpoint
        self.loop_target = loop_target
        self.hysteresis = hysteresis
        self.min_cycles = min_cycles
        self.settle_time_s = settle_time_s
        self.min_peak_spacing_s = min_peak_spacing_s

        self._state = RelayState.IDLE
        self._settle_t0: Optional[float] = None
        self._active_t0: Optional[float] = None

        # Rolling buffers (8 s at 1 kHz)
        self._t_buf:  collections.deque = collections.deque(maxlen=8000)
        self._pv_buf: collections.deque = collections.deque(maxlen=8000)

        self._maxima: List[Tuple[float, float]] = []   # (time_s, value)
        self._minima: List[Tuple[float, float]] = []

        self._result: Optional[ZNGains] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def state(self) -> RelayState:
        return self._state

    @property
    def result(self) -> Optional[ZNGains]:
        return self._result

    def start(self) -> None:
        """Reset and begin settling phase."""
        self._state = RelayState.SETTLING
        self._settle_t0 = None
        self._active_t0 = None
        self._t_buf.clear()
        self._pv_buf.clear()
        self._maxima.clear()
        self._minima.clear()
        self._result = None

    def abort(self) -> None:
        self._state = RelayState.IDLE

    def update(self, sample: TelemetrySample) -> None:
        """
        Feed one sample. Updates `state` and `result` in-place.
        No return value — this is a pure observer.
        """
        if self._state in (RelayState.IDLE, RelayState.DONE, RelayState.FAULT):
            return

        t_s = sample.timestamp_ms / 1000.0
        pv  = self._select_pv(sample)

        # ── SETTLING: discard until settle window elapses ──────────────
        if self._state == RelayState.SETTLING:
            if self._settle_t0 is None:
                self._settle_t0 = t_s
            if (t_s - self._settle_t0) >= self.settle_time_s:
                self._state = RelayState.ACTIVE
                self._active_t0 = t_s
            return

        # ── ACTIVE: accumulate data and detect peaks ───────────────────
        self._t_buf.append(t_s)
        self._pv_buf.append(pv)

        self._detect_peaks()

        # Termination check
        n = min(len(self._maxima), len(self._minima))
        if n >= self.min_cycles:
            gains = self._compute()
            if gains is not None:
                self._result = gains
                self._state = RelayState.DONE
            else:
                self._state = RelayState.FAULT

    def get_plot_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return (times_s, pv_values) suitable for live plotting."""
        if not self._t_buf:
            return np.array([]), np.array([])
        return np.asarray(self._t_buf), np.asarray(self._pv_buf)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _select_pv(self, s: TelemetrySample) -> float:
        return s.velocity_rads if self.loop_target == self.LOOP_VELOCITY else s.position_deg

    def _detect_peaks(self) -> None:
        """
        Incremental 5-point local-extremum detector with minimum spacing guard.
        Runs on the tail of the rolling buffer — O(1) per call.
        """
        if len(self._pv_buf) < 5:
            return

        t_arr  = list(self._t_buf)[-5:]
        pv_arr = list(self._pv_buf)[-5:]
        t_mid  = t_arr[2]
        v_mid  = pv_arr[2]

        # Local maximum
        if v_mid > max(pv_arr[0], pv_arr[1], pv_arr[3], pv_arr[4]):
            if (not self._maxima or
                    (t_mid - self._maxima[-1][0]) >= self.min_peak_spacing_s):
                self._maxima.append((t_mid, v_mid))

        # Local minimum
        elif v_mid < min(pv_arr[0], pv_arr[1], pv_arr[3], pv_arr[4]):
            if (not self._minima or
                    (t_mid - self._minima[-1][0]) >= self.min_peak_spacing_s):
                self._minima.append((t_mid, v_mid))

    def _compute(self) -> Optional[ZNGains]:
        n = min(len(self._maxima), len(self._minima))
        if n < 2:
            return None

        max_vals  = np.array([v for _, v in self._maxima[:n]])
        min_vals  = np.array([v for _, v in self._minima[:n]])
        max_times = np.array([t for t, _ in self._maxima[:n]])
        min_times = np.array([t for t, _ in self._minima[:n]])

        # Oscillation amplitude — robust median over paired half-swings
        amplitudes = (max_vals - min_vals) / 2.0
        a = float(np.median(amplitudes))
        if a < 1e-9:
            return None

        # Ultimate period — median of consecutive peak-to-peak intervals
        periods: List[float] = []
        if len(max_times) >= 2:
            periods.extend(np.diff(max_times).tolist())
        if len(min_times) >= 2:
            periods.extend(np.diff(min_times).tolist())
        if not periods:
            return None
        Pu = float(np.median(periods))
        if Pu <= 0:
            return None

        # Åström-Hägglund
        Ku = (4.0 * self.d) / (np.pi * a)

        # Ziegler-Nichols closed-loop PID
        Kp = 0.60  * Ku
        Ki = 1.20  * Ku / Pu
        Kd = 0.075 * Ku * Pu

        return ZNGains(
            Kp=Kp, Ki=Ki, Kd=Kd,
            Ku=Ku, Pu_s=Pu,
            oscillation_amplitude=a,
            cycle_count=n,
        )
