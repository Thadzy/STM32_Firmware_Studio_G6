#!/usr/bin/env python3
"""
autotune_dashboard.py — PyQt5 + pyqtgraph Auto-Tune Dashboard

Run:
    python autotune_dashboard.py

Dependencies (see requirements_autotune.txt):
    PyQt5, pyqtgraph, numpy, pymodbus>=3.0, websockets>=10.0

Workflow:
  1. Launch serial_bridge.py first (opens the robot UART and WebSocket server).
  2. Start this dashboard.  Click "Connect".
  3. Set relay parameters (amplitude d, setpoint, loop target).
  4. Click "Start Auto-Tune".
  5. The dashboard subscribes to ws://localhost:8765, plots live PV and CO,
     and commands the STM32 into relay mode via Modbus (COM12).
  6. After min_cycles oscillations, Ku/Pu/ZN gains are displayed.
  7. Optionally edit the gains spinboxes, then click "Apply Gains".
"""

from __future__ import annotations

import collections
import logging
import sys
from typing import Optional

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QPalette
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPushButton, QSpinBox, QSplitter, QStatusBar, QTextEdit,
    QVBoxLayout, QWidget,
)

from autotune.worker import AsyncWorker
from autotune.relay_analyzer import RelayState
from autotune.modbus_client import LOOP_POSITION, LOOP_VELOCITY

log = logging.getLogger(__name__)

# ── pyqtgraph global config ─────────────────────────────────────────────────
pg.setConfigOptions(antialias=True, foreground="w", background="#1e1e2e")

PLOT_WINDOW_S  = 30.0    # seconds of history visible on plots
MAX_BUF        = 12000   # 400 Hz × 30 s — plenty of headroom

_STATE_STYLE = {
    RelayState.IDLE:     ("IDLE",     "#888888"),
    RelayState.SETTLING: ("SETTLING", "#f1c40f"),
    RelayState.ACTIVE:   ("ACTIVE",   "#e74c3c"),
    RelayState.DONE:     ("DONE",     "#2ecc71"),
    RelayState.FAULT:    ("FAULT",    "#e74c3c"),
}


# ══════════════════════════════════════════════════════════════════════════════
# Live plot widget
# ══════════════════════════════════════════════════════════════════════════════

class LivePlotWidget(QWidget):
    """
    Two vertically stacked pyqtgraph plots refreshed at ~30 Hz.

    Data is accumulated in deque ring-buffers; the refresh timer converts
    them to NumPy arrays once per frame — O(N) copy, but N ≤ MAX_BUF so it
    is negligible.  The plot window slides to show the last PLOT_WINDOW_S
    seconds at all times.
    """

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        self._t0: Optional[float] = None
        self._t   = collections.deque(maxlen=MAX_BUF)
        self._pv  = collections.deque(maxlen=MAX_BUF)
        self._co  = collections.deque(maxlen=MAX_BUF)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        glw = pg.GraphicsLayoutWidget()
        layout.addWidget(glw)

        # ── PV plot ────────────────────────────────────────────────────
        self._pv_plot = glw.addPlot(row=0, col=0, title="Process Variable")
        self._pv_plot.showGrid(x=True, y=True, alpha=0.25)
        self._pv_plot.setLabel("left", "PV", units="° or rad/s")
        self._pv_plot.setLabel("bottom", "Time", units="s")
        self._pv_curve = self._pv_plot.plot(
            pen=pg.mkPen("#4fc3f7", width=1.5), name="PV"
        )

        # Setpoint reference line
        self._sp_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#f39c12", width=1, style=Qt.DashLine),
            label="SP", labelOpts={"color": "#f39c12"},
        )
        self._pv_plot.addItem(self._sp_line)

        # Hysteresis band shading (visible during relay phase)
        self._hyst_band = pg.LinearRegionItem(
            values=(0.0, 0.0), orientation="horizontal",
            brush=pg.mkBrush(231, 76, 60, 35), pen=pg.mkPen(None), movable=False,
        )
        self._pv_plot.addItem(self._hyst_band)
        self._hyst_band.setVisible(False)

        # ── CO plot ────────────────────────────────────────────────────
        self._co_plot = glw.addPlot(row=1, col=0, title="Control Output")
        self._co_plot.showGrid(x=True, y=True, alpha=0.25)
        self._co_plot.setLabel("left", "CO", units="")
        self._co_plot.setLabel("bottom", "Time", units="s")
        self._co_curve = self._co_plot.plot(
            pen=pg.mkPen("#a5d6a7", width=1.5), name="CO"
        )
        self._co_plot.setXLink(self._pv_plot)

        # Relay reference lines on CO plot
        self._relay_hi = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#e74c3c", width=1, style=Qt.DotLine),
        )
        self._relay_lo = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#3498db", width=1, style=Qt.DotLine),
        )
        self._co_plot.addItem(self._relay_hi)
        self._co_plot.addItem(self._relay_lo)
        self._relay_hi.setVisible(False)
        self._relay_lo.setVisible(False)

        # ── refresh timer (decoupled from data rate) ───────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(33)   # 30 fps

    # ------------------------------------------------------------------

    def set_setpoint(self, sp: float) -> None:
        self._sp_line.setValue(sp)

    def show_relay_overlay(self, center: float, half_hyst: float, amplitude: float) -> None:
        self._hyst_band.setRegion((center - half_hyst, center + half_hyst))
        self._hyst_band.setVisible(True)
        self._relay_hi.setValue(amplitude)
        self._relay_lo.setValue(-amplitude)
        self._relay_hi.setVisible(True)
        self._relay_lo.setVisible(True)

    def hide_relay_overlay(self) -> None:
        self._hyst_band.setVisible(False)
        self._relay_hi.setVisible(False)
        self._relay_lo.setVisible(False)

    def append(self, t_s: float, pv: float, co: float) -> None:
        if self._t0 is None:
            self._t0 = t_s
        self._t.append(t_s - self._t0)
        self._pv.append(pv)
        self._co.append(co)

    def clear(self) -> None:
        self._t0 = None
        self._t.clear()
        self._pv.clear()
        self._co.clear()

    def _refresh(self) -> None:
        if not self._t:
            return
        t   = np.asarray(self._t)
        pv  = np.asarray(self._pv)
        co  = np.asarray(self._co)

        t_max = t[-1]
        mask  = t >= (t_max - PLOT_WINDOW_S)
        t_w, pv_w, co_w = t[mask], pv[mask], co[mask]

        self._pv_curve.setData(t_w, pv_w)
        self._co_curve.setData(t_w, co_w)

        if len(t_w) > 1:
            self._pv_plot.setXRange(float(t_w[0]), float(t_w[-1]), padding=0.02)


# ══════════════════════════════════════════════════════════════════════════════
# Control panel (left column)
# ══════════════════════════════════════════════════════════════════════════════

class ControlPanel(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)

        # ── connection ─────────────────────────────────────────────────
        cg = QGroupBox("Connection")
        cf = QFormLayout(cg)
        self.ws_url   = QLineEdit("ws://localhost:8765")
        self.mb_port  = QLineEdit("COM12")
        self.mb_baud  = QLineEdit("230400")
        cf.addRow("WebSocket:", self.ws_url)
        cf.addRow("Modbus port:", self.mb_port)
        cf.addRow("Baudrate:", self.mb_baud)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.setCheckable(True)
        cf.addRow(self.connect_btn)
        root.addWidget(cg)

        # ── auto-tune parameters ────────────────────────────────────────
        ag = QGroupBox("Auto-Tune Parameters")
        af = QFormLayout(ag)

        self.loop_combo = QComboBox()
        self.loop_combo.addItem("Position loop  (outer, 100 Hz)", LOOP_POSITION)
        self.loop_combo.addItem("Velocity loop  (inner, 1 kHz)",  LOOP_VELOCITY)
        af.addRow("Target loop:", self.loop_combo)

        self.setpoint_sp = _dspin(-360, 360, 90.0, " °",   1)
        self.amplitude_sp = _dspin(0.1,  200, 10.0, " d",   2)
        self.hysteresis_sp = _dspin(0.0,  20,  0.5, " °",   2)
        self.settle_sp   = _dspin(0.5,  30,  2.0, " s",   1)
        self.cycles_sp   = QSpinBox()
        self.cycles_sp.setRange(2, 20)
        self.cycles_sp.setValue(4)

        af.addRow("Setpoint:", self.setpoint_sp)
        af.addRow("Relay amplitude d:", self.amplitude_sp)
        af.addRow("Hysteresis ε:", self.hysteresis_sp)
        af.addRow("Settle time:", self.settle_sp)
        af.addRow("Min cycles:", self.cycles_sp)
        root.addWidget(ag)

        # ── action buttons ──────────────────────────────────────────────
        bg = QGroupBox("Actions")
        bl = QVBoxLayout(bg)
        self.start_btn  = _btn("Start Auto-Tune",      "#27ae60", enabled=False)
        self.abort_btn  = _btn("Abort",                "#c0392b", enabled=False)
        self.apply_btn  = _btn("Apply Computed Gains", "#2980b9", enabled=False)
        bl.addWidget(self.start_btn)
        bl.addWidget(self.abort_btn)
        bl.addWidget(self.apply_btn)
        root.addWidget(bg)

        root.addStretch()

    def set_connected(self, ok: bool) -> None:
        self.connect_btn.setChecked(ok)
        self.connect_btn.setText("Disconnect" if ok else "Connect")
        self.start_btn.setEnabled(ok)


# ══════════════════════════════════════════════════════════════════════════════
# Result panel (right column)
# ══════════════════════════════════════════════════════════════════════════════

class ResultPanel(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)

        # ── state badge ─────────────────────────────────────────────────
        sg = QGroupBox("Analyzer State")
        sl = QVBoxLayout(sg)
        self.state_lbl = QLabel("IDLE")
        self.state_lbl.setAlignment(Qt.AlignCenter)
        f = QFont(); f.setPointSize(20); f.setBold(True)
        self.state_lbl.setFont(f)
        self.state_lbl.setStyleSheet("color: #888888;")
        self.cycles_lbl = QLabel("Cycles collected: 0")
        self.cycles_lbl.setAlignment(Qt.AlignCenter)
        sl.addWidget(self.state_lbl)
        sl.addWidget(self.cycles_lbl)
        root.addWidget(sg)

        # ── identified parameters ───────────────────────────────────────
        pg_ = QGroupBox("Identified Parameters")
        pf  = QFormLayout(pg_)
        self.ku_lbl  = QLabel("—")
        self.pu_lbl  = QLabel("—")
        self.amp_lbl = QLabel("—")
        pf.addRow("Ku (ultimate gain):", self.ku_lbl)
        pf.addRow("Pu (ultimate period):", self.pu_lbl)
        pf.addRow("a  (oscillation amp.):", self.amp_lbl)
        root.addWidget(pg_)

        # ── computed ZN gains (editable before apply) ───────────────────
        gg = QGroupBox("Computed ZN-PID  (editable)")
        gf = QFormLayout(gg)
        self.kp_sp = _dspin(0, 9999, 0.0, "", 4)
        self.ki_sp = _dspin(0, 9999, 0.0, "", 4)
        self.kd_sp = _dspin(0, 9999, 0.0, "", 4)
        gf.addRow("Kp:", self.kp_sp)
        gf.addRow("Ki:", self.ki_sp)
        gf.addRow("Kd:", self.kd_sp)
        root.addWidget(gg)

        # ── current firmware reference (from params.h) ──────────────────
        fg = QGroupBox("Current Firmware Gains  (from params.h)")
        ff = QFormLayout(fg)
        ff.addRow("Kp:", QLabel("1.5800"))
        ff.addRow("Ki:", QLabel("0.5000"))
        ff.addRow("Kd:", QLabel("0.0000"))
        root.addWidget(fg)

        # ── event log ───────────────────────────────────────────────────
        lg = QGroupBox("Event Log")
        ll = QVBoxLayout(lg)
        self.log_te = QTextEdit()
        self.log_te.setReadOnly(True)
        self.log_te.setMaximumHeight(130)
        self.log_te.setFont(QFont("Consolas", 8))
        ll.addWidget(self.log_te)
        root.addWidget(lg)

        root.addStretch()

    def set_state(self, state: RelayState) -> None:
        label, color = _STATE_STYLE.get(state, ("?", "#888888"))
        self.state_lbl.setText(label)
        self.state_lbl.setStyleSheet(f"color: {color};")

    def set_results(
        self, kp: float, ki: float, kd: float,
        ku: float, pu_s: float, a: float, cycles: int,
    ) -> None:
        self.kp_sp.setValue(kp)
        self.ki_sp.setValue(ki)
        self.kd_sp.setValue(kd)
        self.ku_lbl.setText(f"{ku:.4f}")
        self.pu_lbl.setText(f"{pu_s * 1000:.1f} ms")
        self.amp_lbl.setText(f"{a:.3f} °")
        self.cycles_lbl.setText(f"Cycles collected: {cycles}")

    def log(self, msg: str) -> None:
        self.log_te.append(msg)


# ══════════════════════════════════════════════════════════════════════════════
# Main window
# ══════════════════════════════════════════════════════════════════════════════

class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Auto-Tune Dashboard — Pick & Place Robot")
        self.resize(1440, 840)

        self._worker: Optional[AsyncWorker] = None
        self._last_gains: Optional[tuple[float, float, float]] = None

        self._ctrl   = ControlPanel()
        self._plot   = LivePlotWidget()
        self._result = ResultPanel()

        self._ctrl.setFixedWidth(295)
        self._result.setFixedWidth(330)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._ctrl)
        splitter.addWidget(self._plot)
        splitter.addWidget(self._result)
        splitter.setStretchFactor(1, 1)

        container = QWidget()
        QHBoxLayout(container).addWidget(splitter)
        self.setCentralWidget(container)

        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._status.showMessage("Disconnected")

        self._wire()

    # ------------------------------------------------------------------
    # Signal wiring
    # ------------------------------------------------------------------

    def _wire(self) -> None:
        c = self._ctrl
        c.connect_btn.clicked.connect(self._on_connect_toggle)
        c.start_btn.clicked.connect(self._on_start)
        c.abort_btn.clicked.connect(self._on_abort)
        c.apply_btn.clicked.connect(self._on_apply)
        c.setpoint_sp.valueChanged.connect(self._plot.set_setpoint)

    # ------------------------------------------------------------------
    # Slots — user actions
    # ------------------------------------------------------------------

    @pyqtSlot()
    def _on_connect_toggle(self) -> None:
        if self._worker and self._worker.isRunning():
            self._worker.stop()
            self._worker.wait(3000)
            self._worker = None
            self._ctrl.set_connected(False)
            self._status.showMessage("Disconnected")
            return

        self._worker = AsyncWorker(
            ws_url=self._ctrl.ws_url.text().strip(),
            modbus_port=self._ctrl.mb_port.text().strip(),
            modbus_baudrate=int(self._ctrl.mb_baud.text().strip()),
        )
        w = self._worker
        w.signals.telemetry.connect(self._on_telemetry)
        w.signals.analyzer_state.connect(self._on_state_change)
        w.signals.gains_ready.connect(self._on_gains_ready)
        w.signals.modbus_status.connect(self._on_modbus_status)
        w.signals.error.connect(self._on_error)
        w.start()
        self._status.showMessage("Connecting…")

    @pyqtSlot()
    def _on_start(self) -> None:
        if self._worker is None:
            return
        c   = self._ctrl
        sp  = c.setpoint_sp.value()
        d   = c.amplitude_sp.value()
        h   = c.hysteresis_sp.value()
        st  = c.settle_sp.value()
        mc  = c.cycles_sp.value()
        lt  = c.loop_combo.currentData()

        self._plot.set_setpoint(sp)
        self._plot.show_relay_overlay(sp, h / 2.0, d)

        self._worker.start_autotune(d, sp, lt, h, mc, st)

        c.start_btn.setEnabled(False)
        c.abort_btn.setEnabled(True)
        c.apply_btn.setEnabled(False)
        self._result.log(
            f"[START] loop={'pos' if lt == LOOP_POSITION else 'vel'} "
            f"SP={sp}° d={d} ε={h}° settle={st}s min_cycles={mc}"
        )

    @pyqtSlot()
    def _on_abort(self) -> None:
        if self._worker:
            self._worker.abort_autotune()
        self._ctrl.start_btn.setEnabled(True)
        self._ctrl.abort_btn.setEnabled(False)
        self._plot.hide_relay_overlay()
        self._result.log("[ABORT]")

    @pyqtSlot()
    def _on_apply(self) -> None:
        if self._worker is None:
            return
        kp = self._result.kp_sp.value()
        ki = self._result.ki_sp.value()
        kd = self._result.kd_sp.value()
        self._worker.apply_gains(kp, ki, kd)
        self._result.log(f"[APPLY] Kp={kp:.4f}  Ki={ki:.4f}  Kd={kd:.4f}")

    # ------------------------------------------------------------------
    # Slots — worker signals
    # ------------------------------------------------------------------

    @pyqtSlot(float, float, float, float)
    def _on_telemetry(self, t_s: float, pos: float, _vel: float, co: float) -> None:
        self._plot.append(t_s, pos, co)

    @pyqtSlot(int)
    def _on_state_change(self, state_int: int) -> None:
        state = RelayState(state_int)
        self._result.set_state(state)

        if state == RelayState.ACTIVE:
            self._result.log("[RELAY ACTIVE] Collecting oscillations…")

        elif state in (RelayState.DONE, RelayState.FAULT):
            self._ctrl.start_btn.setEnabled(True)
            self._ctrl.abort_btn.setEnabled(False)
            self._plot.hide_relay_overlay()

    @pyqtSlot(float, float, float, float, float, float, int)
    def _on_gains_ready(
        self,
        kp: float, ki: float, kd: float,
        ku: float, pu_s: float, a: float, cycles: int,
    ) -> None:
        self._last_gains = (kp, ki, kd)
        self._result.set_results(kp, ki, kd, ku, pu_s, a, cycles)
        self._ctrl.apply_btn.setEnabled(True)
        self._result.log(
            f"[DONE] Ku={ku:.4f}  Pu={pu_s*1000:.1f} ms  a={a:.3f}°  "
            f"→  Kp={kp:.4f}  Ki={ki:.4f}  Kd={kd:.4f}"
        )

    @pyqtSlot(bool, str)
    def _on_modbus_status(self, ok: bool, msg: str) -> None:
        self._ctrl.set_connected(ok)
        self._status.showMessage(msg)
        self._result.log(f"[MODBUS] {msg}")

    @pyqtSlot(str)
    def _on_error(self, msg: str) -> None:
        self._status.showMessage(f"ERROR: {msg}")
        self._result.log(f"[ERROR] {msg}")

    # ------------------------------------------------------------------

    def closeEvent(self, event) -> None:
        if self._worker and self._worker.isRunning():
            self._worker.stop()
            self._worker.wait(3000)
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════════

def _dspin(lo: float, hi: float, val: float, suffix: str, dec: int) -> QDoubleSpinBox:
    sb = QDoubleSpinBox()
    sb.setRange(lo, hi)
    sb.setValue(val)
    sb.setSuffix(suffix)
    sb.setDecimals(dec)
    sb.setSingleStep(10 ** -dec)
    return sb


def _btn(text: str, bg: str, enabled: bool = True) -> QPushButton:
    b = QPushButton(text)
    b.setEnabled(enabled)
    b.setStyleSheet(f"background-color:{bg}; color:white; font-weight:bold; padding:4px;")
    return b


def _dark_palette() -> QPalette:
    p = QPalette()
    bg     = QColor(30,  30,  46)
    bg2    = QColor(24,  24,  37)
    fg     = QColor(205, 214, 244)
    accent = QColor(137, 180, 250)
    btn    = QColor(49,  50,  68)
    p.setColor(QPalette.Window,          bg)
    p.setColor(QPalette.WindowText,      fg)
    p.setColor(QPalette.Base,            bg2)
    p.setColor(QPalette.AlternateBase,   bg)
    p.setColor(QPalette.ToolTipBase,     bg2)
    p.setColor(QPalette.ToolTipText,     fg)
    p.setColor(QPalette.Text,            fg)
    p.setColor(QPalette.Button,          btn)
    p.setColor(QPalette.ButtonText,      fg)
    p.setColor(QPalette.BrightText,      Qt.red)
    p.setColor(QPalette.Link,            accent)
    p.setColor(QPalette.Highlight,       accent)
    p.setColor(QPalette.HighlightedText, bg)
    return p


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setPalette(_dark_palette())
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
