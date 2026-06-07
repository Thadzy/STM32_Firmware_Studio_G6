class TelemetryChart {
    constructor(containerId, color, minVal, maxVal, tuningMinVal, tuningMaxVal, absInTuning) {
        this.container = document.getElementById(containerId);
        // Replace canvas with div if needed
        if (this.container.tagName === 'CANVAS') {
            const div = document.createElement('div');
            div.id = this.container.id;
            div.className = this.container.className;
            div.style.width = '100%';
            div.style.height = '100%';
            this.container.parentNode.replaceChild(div, this.container);
            this.container = div;
        }

        this.color = color || '#00f2ff';
        this.minVal = (minVal !== undefined && minVal !== null) ? minVal : -10;
        this.maxVal = (maxVal !== undefined && maxVal !== null) ? maxVal : 360;
        this.tuningMinVal = tuningMinVal;
        this.tuningMaxVal = tuningMaxVal;
        this.absInTuning = !!absInTuning;
        this.tuningMode = false;
        
        this.maxDataPoints = 200;
        this.times = [];
        this.vals = [];
        this.tgts = [];
        this.ests = [];
        this.sans = [];
        
        const opts = {
            width: this.container.clientWidth || 400,
            height: this.container.clientHeight || 200,
            axes: [
                { show: false },
                { stroke: 'rgba(255,255,255,0.4)', grid: { stroke: 'rgba(255,255,255,0.05)', width: 1 } }
            ],
            scales: {
                x: { time: false },
                y: { auto: false, range: [this.minVal, this.maxVal] }
            },
            series: [
                {},
                { stroke: this.color, width: 2, points: { show: false } },
                { stroke: 'rgba(255,255,255,0.4)', width: 1.5, dash: [5, 5], points: { show: false } },
                { stroke: '#00ff88', width: 1.5, points: { show: false } },
                { stroke: 'rgba(255,200,0,0.7)', width: 1.5, dash: [5, 5], points: { show: false } }
            ],
            legend: { show: false },
            cursor: { show: false }
        };
        
        this.uplot = new uPlot(opts, [[],[],[],[],[]], this.container);
        
        window.addEventListener('resize', () => this.resize());
        setTimeout(() => this.resize(), 100);
        
        this.tuningRuns = [];
        this.currentRun = null;
        this.previewSine = null;
    }

    resize() {
        const w = Math.max(10, this.container.clientWidth);
        const h = Math.max(10, this.container.clientHeight);
        this.uplot.setSize({ width: w, height: h });
    }

    setMode(tuning) {
        this.tuningMode = tuning;
        const minV = tuning && this.tuningMinVal !== undefined ? this.tuningMinVal : this.minVal;
        const maxV = tuning && this.tuningMaxVal !== undefined ? this.tuningMaxVal : this.maxVal;
        
        this.uplot.setScale('y', { min: minV, max: maxV });
        
        // Show/hide X axis depending on mode
        this.uplot.axes[0].show = tuning;
        
        this.draw();
    }

    addData(val) {
        this.vals.push(val);
        this.tgts.push(this._lastTgt !== undefined ? this._lastTgt : null);
        this.ests.push(this._lastEst !== undefined ? this._lastEst : null);
        this.sans.push(this._lastSan !== undefined ? this._lastSan : null);
        
        if (this._t === undefined) this._t = 0;
        this.times.push(this._t);
        this._t += 0.01; // 100Hz = 10ms per tick
        
        if (this.vals.length > this.maxDataPoints) {
            this.vals.shift();
            this.tgts.shift();
            this.ests.shift();
            this.sans.shift();
            this.times.shift();
        }
        
        if (!this.tuningMode) this.draw();
    }

    addTarget(val) {
        this._lastTgt = val;
        if (this.tgts.length > 0) this.tgts[this.tgts.length - 1] = val;
    }

    addEstimate(val) {
        this._lastEst = val;
        if (this.ests.length > 0) this.ests[this.ests.length - 1] = val;
    }

    addSanity(val) {
        this._lastSan = val;
        if (this.sans.length > 0) this.sans[this.sans.length - 1] = val;
    }

    clearEstimate() { this.ests.fill(null); this._lastEst = null; }
    clearSanity() { this.sans.fill(null); this._lastSan = null; }

    startRun(target) {
        this.currentRun = { times: [], vals: [], vsets: [], target, startTime: Date.now() };
    }

    addRunPoint(val, vset) {
        if (!this.currentRun) return;
        const t = (Date.now() - this.currentRun.startTime) / 1000;
        this.currentRun.times.push(t);
        this.currentRun.vals.push(this.absInTuning ? Math.abs(val) : val);
        this.currentRun.vsets.push(vset !== undefined ? (this.absInTuning ? Math.abs(vset) : vset) : 0);
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

    setPreviewSine(amp, freq) {
        this.previewSine = (amp && freq > 0) ? { amp, freq } : null;
        this.draw();
    }
    
    clearPreviewSine() {
        this.previewSine = null;
        this.draw();
    }

    draw() {
        if (!this._pendingDraw) {
            this._pendingDraw = true;
            requestAnimationFrame(() => {
                this._pendingDraw = false;
                this._doDraw();
            });
        }
    }

    _doDraw() {
        if (this.tuningMode) {
            let times = [];
            let vals = [];
            let vsets = [];
            
            // Build data from runs
            const lastIdx = this.tuningRuns.length - 1;
            
            if (this.currentRun) {
                times = times.concat(this.currentRun.times);
                vals = vals.concat(this.currentRun.vals);
                vsets = vsets.concat(this.currentRun.vsets);
            } else if (this.tuningRuns.length > 0) {
                const run = this.tuningRuns[lastIdx];
                times = times.concat(run.times);
                vals = vals.concat(run.vals);
                vsets = vsets.concat(run.vsets);
            }
            
            if (times.length > 0) {
                this.uplot.setData([times, vals, vsets, [], []]);
            }
        } else {
            if (this.times.length > 0 && this.vals.length === this.times.length) {
                this.uplot.setData([
                    this.times,
                    this.vals,
                    this.tgts,
                    this.ests,
                    this.sans
                ]);
            }
        }
    }
}
