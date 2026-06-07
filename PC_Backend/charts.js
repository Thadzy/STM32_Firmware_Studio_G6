class TelemetryChart {
    constructor(canvasId, color, minVal, maxVal, tuningMinVal, tuningMaxVal, absInTuning) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext('2d');
        this.history = [];
        this.targetHistory = [];
        this.maxDataPoints = 200;
        this.color = color || '#00f2ff';
        this.minVal = (minVal !== undefined && minVal !== null) ? minVal : -10;
        this.maxVal = (maxVal !== undefined && maxVal !== null) ? maxVal : 360;
        // Optional Y-axis range override for tuning mode only
        this.tuningMinVal = tuningMinVal;
        this.tuningMaxVal = tuningMaxVal;
        this.absInTuning  = !!absInTuning;

        // Tuning mode
        this.tuningMode = false;
        this.tuningRuns = [];        // [{times[], vals[], vsets[], target, settleTime, overshoot}]
        this.currentRun = null;

        window.addEventListener('resize', () => this.resize());
        this.resize();
    }

    resize() {
        // Use the canvas's own laid-out size (CSS-driven) rather than the
        // parent's clientHeight. This avoids the feedback loop where the
        // canvas's drawing-buffer size grows the flex item, which grows
        // the parent, which grows the buffer again on the next toggle.
        const rect = this.canvas.getBoundingClientRect();
        const w = Math.max(1, Math.round(rect.width));
        const h = Math.max(1, Math.round(rect.height));
        if (this.canvas.width !== w || this.canvas.height !== h) {
            this.canvas.width = w;
            this.canvas.height = h;
        }
        this.draw();
    }

    setMode(tuning) {
        this.tuningMode = tuning;
        this.draw();
    }

    // Helpers: effective Y range in tuning mode
    _effMin() {
        return (this.tuningMode && this.tuningMinVal !== undefined) ? this.tuningMinVal : this.minVal;
    }
    _effMax() {
        return (this.tuningMode && this.tuningMaxVal !== undefined) ? this.tuningMaxVal : this.maxVal;
    }
    _xform(v) {
        return (this.tuningMode && this.absInTuning) ? Math.abs(v) : v;
    }

    // --- Live mode ---
    addData(val) {
        if (this.canvas.width === 0 || this.canvas.height === 0) this.resize();
        this.history.push({ value: val });
        if (this.history.length > this.maxDataPoints) this.history.shift();
        if (!this.tuningMode) this.draw();
    }

    addTarget(val) {
        this.targetHistory.push({ value: val });
        if (this.targetHistory.length > this.maxDataPoints) this.targetHistory.shift();
    }

    /* Overlay traces — used by the Kalman dashboard.
     * estHistory: KF state estimate.
     * sanityHistory: open-loop physical model prediction (no correction). */
    addEstimate(val) {
        if (!this.estHistory) this.estHistory = [];
        this.estHistory.push({ value: val });
        if (this.estHistory.length > this.maxDataPoints) this.estHistory.shift();
    }
    addSanity(val) {
        if (!this.sanityHistory) this.sanityHistory = [];
        this.sanityHistory.push({ value: val });
        if (this.sanityHistory.length > this.maxDataPoints) this.sanityHistory.shift();
    }
    clearEstimate() { this.estHistory = []; }
    clearSanity()   { this.sanityHistory = []; }

    // --- Tuning mode ---
    startRun(target) {
        this.currentRun = { times: [], vals: [], vsets: [], target, startTime: Date.now() };
    }

    addRunPoint(val, vset) {
        if (!this.currentRun) return;
        const t = (Date.now() - this.currentRun.startTime) / 1000;
        this.currentRun.times.push(t);
        this.currentRun.vals.push(val);
        this.currentRun.vsets.push(vset !== undefined ? vset : 0);
        if (this.tuningMode) this.draw();
    }

    finalizeRun(settleTime, overshoot) {
        if (!this.currentRun) return;
        this.tuningRuns.push({ ...this.currentRun, settleTime, overshoot });
        if (this.tuningRuns.length > 3) this.tuningRuns.shift();
        this.currentRun = null;
        if (this.tuningMode) this.draw();
    }

    clearRuns() {
        this.tuningRuns = [];
        this.currentRun = null;
        this.draw();
    }

    // --- Shared draw ---
    draw() {
        if (this.tuningMode) this._drawTuning();
        else this._drawLive();
    }

    _drawLine(history, color, dashed, lineWidth) {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const range = this.maxVal - this.minVal;
        const xStep = width / (this.maxDataPoints - 1);
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = lineWidth || 2;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]);
        else ctx.setLineDash([]);
        history.forEach((point, i) => {
            const x = i * xStep;
            let val = Math.min(Math.max(point.value, this.minVal), this.maxVal);
            const y = height - ((val - this.minVal) / range) * height;
            if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    setPreviewSine(amp, freq) {
        this.previewSine = (amp && freq > 0) ? { amp, freq } : null;
        this.draw();
    }
    clearPreviewSine() { this.previewSine = null; this.draw(); }

    _drawSinePreview(width, height, minV, maxV) {
        if (!this.previewSine) return;
        const { amp, freq } = this.previewSine;
        const range = maxV - minV;
        const ctx = this.ctx;
        const periods = 2;
        ctx.beginPath();
        ctx.strokeStyle = '#ff3131';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        for (let x = 0; x <= width; x++) {
            const phase = (x / width) * periods * 2 * Math.PI;
            const val = Math.min(Math.max(amp * Math.sin(phase), minV), maxV);
            const y = height - ((val - minV) / range) * height;
            if (x === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = '#ff3131';
        ctx.font = '10px "JetBrains Mono"';
        ctx.fillText(`PREVIEW  ±${amp} RPM @ ${freq} Hz`, 6, 14);
    }

    _drawLive() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const range = this.maxVal - this.minVal;
        ctx.clearRect(0, 0, width, height);
        this._drawGrid(width, height, range);

        if (this.history.length < 2) return;
        if (this.targetHistory.length >= 2)
            this._drawLine(this.targetHistory, 'rgba(255,255,255,0.35)', true, 1.5);
        this._drawLine(this.history, this.color, false, 2);
        if (this.estHistory    && this.estHistory.length    >= 2)
            this._drawLine(this.estHistory,    '#00ff88', false, 1.8);   /* KF estimate (lime) */
        if (this.sanityHistory && this.sanityHistory.length >= 2)
            this._drawLine(this.sanityHistory, 'rgba(255,200,0,0.7)', true, 1.5); /* model sanity (amber dash) */

        ctx.fillStyle = this.color;
        ctx.font = 'bold 11px "JetBrains Mono"';
        const lastVal = this.history[this.history.length - 1].value;
        const text = isNaN(lastVal) ? '--' : lastVal.toFixed(1);
        const tw = ctx.measureText(text).width;
        ctx.fillText(text, width - tw - 5, 15);

        if (this.targetHistory.length > 0) {
            const lt = this.targetHistory[this.targetHistory.length - 1].value;
            const tt = 'T:' + (isNaN(lt) ? '--' : lt.toFixed(1));
            ctx.fillStyle = 'rgba(255,255,255,0.5)';
            ctx.font = '9px "JetBrains Mono"';
            ctx.fillText(tt, width - ctx.measureText(tt).width - 5, 28);
        }
    }

    _drawTuning() {
        const { width, height } = this.canvas;
        const ctx = this.ctx;
        const minV = this._effMin();
        const maxV = this._effMax();
        const range = maxV - minV;
        ctx.clearRect(0, 0, width, height);

        // Compute time window. Two modes:
        //  - Scroll mode (this.scrollWindowSec set): a fixed window of N
        //    seconds that slides with the live signal. Used by the sine test
        //    so the wave doesn't get squeezed as the capture runs long.
        //  - Fit mode (default): zoom out to cover the entire run, including
        //    ghost runs in history.
        let minT = 0;
        let maxT = 10;
        const lastT = (this.currentRun && this.currentRun.times.length)
            ? this.currentRun.times[this.currentRun.times.length - 1]
            : 0;
        const scrolling = this.scrollWindowSec && this.scrollWindowSec > 0;
        if (scrolling) {
            const win = this.scrollWindowSec;
            maxT = Math.max(win, lastT);
            minT = Math.max(0, maxT - win);
        } else {
            this.tuningRuns.forEach(r => { if (r.times.length) maxT = Math.max(maxT, r.times[r.times.length - 1] + 1); });
            if (this.currentRun && this.currentRun.times.length)
                maxT = Math.max(maxT, lastT + 1);
        }

        this._drawGrid(width, height, range);
        this._drawSinePreview(width, height, minV, maxV);

        // Time axis labels
        ctx.fillStyle = 'rgba(255,255,255,0.4)';
        ctx.font = '9px "JetBrains Mono"';
        const span = maxT - minT;
        const step = span <= 6 ? 1 : (span <= 20 ? 2 : 5);
        for (let s = Math.ceil(minT / step) * step; s <= maxT; s += step) {
            const x = ((s - minT) / span) * width;
            ctx.fillText(s + 's', x + 2, height - 2);
        }

        // Ghost runs. Most recent gets a clearly visible alpha so it can be
        // compared against the live trace; older runs fade out. Index 0 is
        // the oldest run, length-1 is the most recent.
        const lastIdx = this.tuningRuns.length - 1;
        const alphaFor = (ri) => (ri === lastIdx) ? 0.55 : (ri === lastIdx - 1 ? 0.25 : 0.10);
        if (!scrolling) this.tuningRuns.forEach((run, ri) => {
            const alpha = alphaFor(ri);
            const col = this._hexToRgba(this.color, alpha);
            this._drawTuningLine(run.times, run.vals, maxT, col, false, 1.5, width, height, range, minV, maxV, minT);

            // Target line for this run
            if (run.target !== undefined) {
                const t = this._xform(run.target);
                const ty = height - ((Math.min(Math.max(t, minV), maxV) - minV) / range) * height;
                ctx.beginPath();
                ctx.strokeStyle = `rgba(255,255,255,${alpha * 1.5})`;
                ctx.setLineDash([4, 6]);
                ctx.lineWidth = 1;
                ctx.moveTo(0, ty); ctx.lineTo(width, ty);
                ctx.stroke();
                ctx.setLineDash([]);
            }

            // Settling marker
            if (run.settleTime !== null && run.settleTime !== undefined) {
                const sx = ((run.settleTime - minT) / span) * width;
                ctx.beginPath();
                ctx.strokeStyle = `rgba(100,255,100,${alpha * 2})`;
                ctx.setLineDash([3, 4]);
                ctx.lineWidth = 1;
                ctx.moveTo(sx, 0); ctx.lineTo(sx, height);
                ctx.stroke();
                ctx.setLineDash([]);
            }
        });

        // Current run (bright)
        if (this.currentRun) {
            // Target bar (bright magenta dashed)
            if (this.currentRun.target !== undefined && this.currentRun.target !== 0) {
                const ct = this._xform(this.currentRun.target);
                const ty = height - ((Math.min(Math.max(ct, minV), maxV) - minV) / range) * height;
                ctx.beginPath();
                ctx.strokeStyle = '#ff4ddd';
                ctx.setLineDash([8, 5]);
                ctx.lineWidth = 1.5;
                ctx.moveTo(0, ty); ctx.lineTo(width, ty);
                ctx.stroke();
                ctx.setLineDash([]);
                // Target label
                ctx.fillStyle = '#ff4ddd';
                ctx.font = 'bold 10px "JetBrains Mono"';
                const lbl = 'TARGET ' + this.currentRun.target.toFixed(1);
                ctx.fillText(lbl, 8, ty - 4);
            }
            if (this.currentRun.times.length >= 2) {
                // Setpoint reference (dashed white) — sine wave or step velocity command
                if (this.currentRun.vsets && this.currentRun.vsets.length >= 2) {
                    this._drawTuningLine(this.currentRun.times, this.currentRun.vsets, maxT,
                                         'rgba(255,255,255,0.55)', true, 1.5,
                                         width, height, range, minV, maxV, minT);
                }
                // Actual value (solid, channel color)
                this._drawTuningLine(this.currentRun.times, this.currentRun.vals, maxT, this.color, false, 2, width, height, range, minV, maxV, minT);
            }

            // Current value label
            if (this.currentRun.vals.length > 0) {
                const lastV = this.currentRun.vals[this.currentRun.vals.length - 1];
                ctx.fillStyle = this.color;
                ctx.font = 'bold 11px "JetBrains Mono"';
                const ltext = isNaN(lastV) ? '--' : lastV.toFixed(1);
                ctx.fillText(ltext, width - ctx.measureText(ltext).width - 5, 15);
            }
        }

        // Scale labels
        ctx.fillStyle = 'rgba(255,255,255,0.5)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.fillText(maxV, 5, 10);
        ctx.fillText(minV, 5, height - 14);

        // "TUNING" watermark
        ctx.fillStyle = 'rgba(255,200,0,0.08)';
        ctx.font = 'bold 28px "JetBrains Mono"';
        ctx.fillText('TUNING', 8, height / 2 + 10);
    }

    _drawTuningLine(times, vals, maxT, color, dashed, lw, width, height, range, minV, maxV, minT) {
        if (times.length < 2) return;
        if (minV === undefined) minV = this.minVal;
        if (maxV === undefined) maxV = this.maxVal;
        if (minT === undefined) minT = 0;
        const span = maxT - minT;
        if (span <= 0) return;
        const ctx = this.ctx;
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = lw;
        ctx.lineJoin = 'round';
        if (dashed) ctx.setLineDash([6, 4]); else ctx.setLineDash([]);
        let started = false;
        times.forEach((t, i) => {
            if (t < minT) return;          // skip samples to the left of the window
            const x = ((t - minT) / span) * width;
            let val = Math.min(Math.max(this._xform(vals[i]), minV), maxV);
            const y = height - ((val - minV) / range) * height;
            if (!started) { ctx.moveTo(x, y); started = true; }
            else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    _drawGrid(width, height, range) {
        const ctx = this.ctx;
        const minV = this._effMin();
        const maxV = this._effMax();
        const yRange = maxV - minV;

        ctx.lineWidth = 1;

        // Horizontal grid lines + Y-axis tick labels
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.05)';
        for (let i = 0; i <= 4; i++) {
            const y = (height / 4) * i;
            ctx.moveTo(0, y);
            ctx.lineTo(width, y);
        }
        ctx.stroke();

        ctx.fillStyle = 'rgba(255,255,255,0.45)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.textBaseline = 'middle';
        for (let i = 0; i <= 4; i++) {
            const y = (height / 4) * i;
            const val = maxV - (i / 4) * yRange;
            const txt = Math.abs(val) >= 100 ? val.toFixed(0) : val.toFixed(1);
            // Nudge top/bottom labels inward so they aren't clipped
            const yT = i === 0 ? y + 7 : (i === 4 ? y - 7 : y);
            ctx.fillText(txt, 4, yT);
        }

        // Left axis line
        ctx.beginPath();
        ctx.strokeStyle = 'rgba(255,255,255,0.3)';
        ctx.moveTo(1, 0); ctx.lineTo(1, height);
        ctx.stroke();

        // Zero line (highlighted)
        const zeroY = height - ((0 - minV) / yRange) * height;
        if (zeroY >= 0 && zeroY <= height) {
            ctx.beginPath();
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.moveTo(0, zeroY); ctx.lineTo(width, zeroY);
            ctx.stroke();
            ctx.fillStyle = 'rgba(255,255,255,0.55)';
            ctx.fillText('0', 4, zeroY - 1);
        }

        // X-axis label (right edge) — "now" indicator for live, "s" for tuning
        ctx.fillStyle = 'rgba(255,255,255,0.35)';
        ctx.font = '9px "JetBrains Mono"';
        ctx.textBaseline = 'alphabetic';
        const xLbl = this.tuningMode ? 'time (s) →' : 'now →';
        const w = ctx.measureText(xLbl).width;
        ctx.fillText(xLbl, width - w - 4, height - 4);
    }

    _hexToRgba(hex, alpha) {
        const c = hex.replace('#', '');
        if (c.length === 6) {
            const r = parseInt(c.slice(0,2),16), g = parseInt(c.slice(2,4),16), b = parseInt(c.slice(4,6),16);
            return `rgba(${r},${g},${b},${alpha})`;
        }
        return hex;
    }
}
