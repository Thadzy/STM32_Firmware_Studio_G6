/* ===========================================================
 * testing.js — Test Suite for ROBO-COOK dashboard
 * Buffers live telemetry into CSV files (raw data only — no analysis).
 *
 * Depends on app.js (uses globals: state, sendCommand, log).
 * Saves to a user-picked directory via File System Access API,
 * falls back to browser downloads if no directory granted.
 * =========================================================== */
(function () {
    'use strict';

    // ---- Config: columns captured per telemetry packet ----
    // Mirror the keys parsed in app.js processPacket().
    const COLUMNS = [
        't_ms',        // PC-side capture timestamp (ms since recording start)
        'wall_ms',     // PC wall-clock ms since epoch
        'pos',         // POS  — encoder position (deg)
        'target',      // TAR  — firmware target (deg)
        'vel',         // VEL  — velocity (RPM)
        'vel_set',     // VSET — velocity setpoint
        'acc',         // ACC  — acceleration
        'acc_set',     // ASET — acceleration setpoint
        'pwm',         // PWM  — applied PWM %
        'mode',        // MODE — decoded mode string
        'fault',       // FAULT decoded
        'kf_theta',    // KTH  — kalman θ (deg)
        'kf_omega',    // KOM  — kalman ω (RPM)
        'kf_tau',      // KTL  — kalman τ_L
        'kf_ia',       // KIA  — kalman î_a (A)  ← used for L4-6
        'kf_innov',    // KIV
        'kf_sanity_th',// KSTH — model θ
        'kf_sanity_om',// KSOM — model ω
        'kf_enabled',  // KFEN
        'estop',       // ESTOP
        'prox',        // PROX
    ];

    // ---- State ----
    const T = {
        dirHandle: null,         // FileSystemDirectoryHandle (or null → downloads)
        recording: false,
        testId: null,
        params: '',
        rows: [],
        startTs: 0,
        startedAt: null,         // Date
        timerId: null,
        recentFiles: [],         // {name, samples, date}
        trialCounters: {},       // {HW3: 3, ...}
        // Capture limits (read at startRecording time)
        limits: null,            // {timeMs, settleRpm, decim} or null
        decimCounter: 0,
        settleUnderSince: 0,     // ms since velocity went under threshold (0 = not under)
        autoStopTimerId: null,
    };

    // ---- Test catalogue: button id → {prompts, cmd builder, suggestParam} ----
    // The `cmd` is what we send via $...* to firmware (the firmware-side
    // $TEST,... parser is added in a separate commit). For tests that don't
    // have a firmware sequence yet, set cmd: null — we just record while the
    // user drives the robot from existing dashboard controls.
    const TESTS = {
        'HW1':  { label: 'HW-1 Encoder',        prompts: ['Target angle (deg)'],    cmd: (p) => null },
        'HW2':  { label: 'HW-2 PWM→Speed',      prompts: ['Duty %'],                cmd: (p) => `TEST,HW2,${p[0]}` },
        'HW3':  { label: 'HW-3 Step (no load)', prompts: ['Step target (deg)'],     cmd: (p) => `TEST,HW3,${p[0]}` },
        'HW4':  { label: 'HW-4 Step (2 kg)',    prompts: ['Step target (deg)'],     cmd: (p) => `TEST,HW4,${p[0]}` },
        'HW5':  { label: 'HW-5 Precision 100×', prompts: ['Target (deg)', 'Cycles'],cmd: (p) => `TEST,HW5,${p[0]},${p[1]||100}` },
        'HW6':  { label: 'HW-6 Rot 540°',       prompts: [],                        cmd: ()  => `TEST,HW6` },
        'HW7':  { label: 'HW-7 Full Auto',      prompts: [],                        cmd: ()  => `TEST,HW7` },
        'HW8':  { label: 'HW-8 Set Home',       prompts: ['Set-home angle (deg)'],  cmd: (p) => null },
        'HW9':  { label: 'HW-9 Out-of-WS',      prompts: ['Out-of-range target'],   cmd: (p) => `TEST,HW9,${p[0]}` },
        'HW10': { label: 'HW-10 Rod drop',      prompts: ['Release angle (deg)'],   cmd: (p) => null },
        'L4-1-TRAP':   { label: 'L4-1 Trap',     prompts: ['Target (deg)'], cmd: (p) => `TEST,L41,TRAP,${p[0]}` },
        'L4-1-SCURVE': { label: 'L4-1 S-Curve',  prompts: ['Target (deg)'], cmd: (p) => `TEST,L41,SCURVE,${p[0]}` },
        'L4-2-FF':     { label: 'L4-2 +FF',      prompts: ['Target (deg)'], cmd: (p) => `TEST,L42,FF,${p[0]}` },
        'L4-2-NOFF':   { label: 'L4-2 −FF',      prompts: ['Target (deg)'], cmd: (p) => `TEST,L42,NOFF,${p[0]}` },
        'L4-3':        { label: 'L4-3 PID step', prompts: ['Target (deg)'], cmd: (p) => `TEST,L43,${p[0]}` },
        'L4-4':        { label: 'L4-4 Anti-windup', prompts: ['Target (deg)', 'AWU (1/0)'], cmd: (p) => `TEST,L44,${p[0]},${p[1]||1}` },
        'L4-5':        { label: 'L4-5 Kalman Q/R',  prompts: ['Target (deg)'], cmd: (p) => `TEST,L45,${p[0]}` },
        'L4-6':        { label: 'L4-6 Kalman ia',   prompts: ['Target (deg)'], cmd: (p) => `TEST,L46,${p[0]}` },
        'L4-7':        { label: 'L4-7 SysID',       prompts: [],               cmd: ()  => `TEST,L47` },
    };

    // ---- DOM helpers ----
    const $ = (id) => document.getElementById(id);
    function dlog(msg) {
        if (typeof log === 'function') log('[TEST] ' + msg);
        else console.log('[TEST]', msg);
    }

    // ---- Folder picker ----
    async function pickFolder() {
        if (!('showDirectoryPicker' in window)) {
            dlog('File System Access API unavailable — CSVs will download as files.');
            $('test-folder-status').textContent = 'Browser does not support folder picker — using downloads.';
            return;
        }
        try {
            const handle = await window.showDirectoryPicker({ mode: 'readwrite' });
            T.dirHandle = handle;
            const st = $('test-folder-status');
            st.textContent = '📁 ' + handle.name + '/ (auto-save active)';
            st.classList.add('active');
            dlog('Folder selected: ' + handle.name);
        } catch (e) {
            dlog('Folder pick cancelled: ' + e.message);
        }
    }

    // ---- Capture limits ----
    function readLimits() {
        const lim = { timeMs: null, settleRpm: null, decim: 1 };
        if ($('cap-opt-time') && $('cap-opt-time').checked) {
            const v = parseFloat($('cap-opt-time-val').value);
            if (v > 0) lim.timeMs = v * 1000;
        }
        if ($('cap-opt-settle') && $('cap-opt-settle').checked) {
            const v = parseFloat($('cap-opt-settle-val').value);
            if (v > 0) lim.settleRpm = v;
        }
        if ($('cap-opt-decim') && $('cap-opt-decim').checked) {
            const v = parseInt($('cap-opt-decim-val').value, 10);
            if (v > 1) lim.decim = v;
        }
        const active = (lim.timeMs || lim.settleRpm || lim.decim > 1);
        return active ? lim : null;
    }

    // ---- Recording ----
    function startRecording(testId, params, sendCmd) {
        if (T.recording) {
            dlog('Already recording — stop first.');
            return;
        }
        T.recording = true;
        T.testId = testId;
        T.params = (params || []).join('_');
        T.rows = [];
        T.startTs = performance.now();
        T.startedAt = new Date();
        T.trialCounters[testId] = (T.trialCounters[testId] || 0) + 1;

        // Apply capture limits
        T.limits = readLimits();
        T.decimCounter = 0;
        T.settleUnderSince = 0;
        if (T.autoStopTimerId) { clearTimeout(T.autoStopTimerId); T.autoStopTimerId = null; }
        if (T.limits && T.limits.timeMs) {
            T.autoStopTimerId = setTimeout(() => {
                if (T.recording) {
                    dlog('Auto-stop (time limit reached).');
                    stopRecording();
                }
            }, T.limits.timeMs);
        }
        if (T.limits) {
            const bits = [];
            if (T.limits.timeMs) bits.push(`stop@${(T.limits.timeMs/1000).toFixed(1)}s`);
            if (T.limits.settleRpm) bits.push(`settle<${T.limits.settleRpm}RPM`);
            if (T.limits.decim > 1) bits.push(`1in${T.limits.decim}`);
            dlog('Limits: ' + bits.join(', '));
        }

        $('test-recording-bar').classList.remove('hidden');
        $('test-rec-label').textContent = 'Recording ' + testId + (T.params ? ' (' + T.params + ')' : '');
        document.querySelectorAll('.test-btn').forEach(b => {
            b.disabled = (b.dataset.test !== testId);
            if (b.dataset.test === testId) b.classList.add('running');
        });

        if (sendCmd && typeof sendCommand === 'function') {
            sendCommand(sendCmd);
            dlog('Sent: $' + sendCmd + '*');
        }

        if (T.timerId) clearInterval(T.timerId);
        T.timerId = setInterval(updateRecBar, 100);
    }

    function updateRecBar() {
        const elapsed = (performance.now() - T.startTs) / 1000;
        $('test-rec-elapsed').textContent = elapsed.toFixed(1) + 's';
        $('test-rec-samples').textContent = T.rows.length + ' samples';
    }

    async function stopRecording() {
        if (!T.recording) return;
        T.recording = false;
        if (T.timerId) { clearInterval(T.timerId); T.timerId = null; }
        if (T.autoStopTimerId) { clearTimeout(T.autoStopTimerId); T.autoStopTimerId = null; }
        $('test-recording-bar').classList.add('hidden');
        document.querySelectorAll('.test-btn').forEach(b => {
            b.disabled = false;
            b.classList.remove('running');
        });

        if (T.rows.length === 0) {
            dlog('No samples captured — nothing saved.');
            return;
        }

        const fname = buildFilename(T.testId, T.params, T.startedAt, T.trialCounters[T.testId]);
        const csv = rowsToCsv(T.rows);
        await saveCsv(fname, csv);
        addRecentFile(fname, T.rows.length, T.startedAt);
        dlog('Saved ' + fname + ' (' + T.rows.length + ' samples)');
    }

    // ---- Snapshot per telemetry packet (called from app.js) ----
    function onPacket(stateObj /* , rawPacket */) {
        if (!T.recording) return;
        const now = performance.now();
        const t_ms = (now - T.startTs);

        // Motion gate — skip rows while velocity is below threshold,
        // but keep recording open. Single trial can capture multiple moves.
        if (T.limits && T.limits.settleRpm) {
            const absVel = Math.abs(stateObj.vel || 0);
            if (absVel < T.limits.settleRpm) return;
        }

        // Decimation — keep 1 in N packets
        if (T.limits && T.limits.decim > 1) {
            T.decimCounter = (T.decimCounter + 1) % T.limits.decim;
            if (T.decimCounter !== 1) return;
        }

        T.rows.push({
            t_ms: t_ms.toFixed(2),
            wall_ms: Date.now(),
            pos: stateObj.currentPos,
            target: stateObj.firmwareTarget,
            vel: stateObj.vel,
            vel_set: stateObj.velSetpoint,
            acc: stateObj.acc,
            acc_set: stateObj.accSetpoint,
            pwm: stateObj.pwm,
            mode: stateObj.mode,
            fault: stateObj.fault,
            kf_theta: stateObj.kfTheta,
            kf_omega: stateObj.kfOmega,
            kf_tau: stateObj.kfTau,
            kf_ia: stateObj.kfIa,
            kf_innov: stateObj.kfInnov,
            kf_sanity_th: stateObj.kfSanityTheta,
            kf_sanity_om: stateObj.kfSanityOmega,
            kf_enabled: stateObj.kfEnabled ? 1 : 0,
            estop: stateObj.estop ? 1 : 0,
            prox: stateObj.prox ? 1 : 0,
        });
    }

    // ---- CSV build / save ----
    function rowsToCsv(rows) {
        const lines = [COLUMNS.join(',')];
        for (const r of rows) {
            lines.push(COLUMNS.map(c => fmt(r[c])).join(','));
        }
        return lines.join('\n') + '\n';
    }
    function fmt(v) {
        if (v === undefined || v === null) return '';
        if (typeof v === 'number') return Number.isFinite(v) ? v : '';
        // strings: escape commas/quotes
        const s = String(v);
        return /[",\n]/.test(s) ? '"' + s.replace(/"/g, '""') + '"' : s;
    }

    function pad(n, w) { return String(n).padStart(w, '0'); }
    function timestampStr(d) {
        return d.getFullYear() + pad(d.getMonth()+1,2) + pad(d.getDate(),2)
             + '_' + pad(d.getHours(),2) + pad(d.getMinutes(),2) + pad(d.getSeconds(),2);
    }
    function buildFilename(testId, params, date, trial) {
        const safe = (testId + (params ? '_' + params : '')).replace(/[^A-Za-z0-9_-]/g, '_');
        return safe + '_trial' + pad(trial, 3) + '_' + timestampStr(date) + '.csv';
    }

    async function saveCsv(filename, content) {
        if (T.dirHandle) {
            try {
                const fh = await T.dirHandle.getFileHandle(filename, { create: true });
                const ws = await fh.createWritable();
                await ws.write(content);
                await ws.close();
                return;
            } catch (e) {
                dlog('Folder write failed (' + e.message + ') — falling back to download.');
            }
        }
        // Fallback: trigger browser download
        const blob = new Blob([content], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url; a.download = filename;
        document.body.appendChild(a); a.click(); a.remove();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    function addRecentFile(name, samples, date) {
        T.recentFiles.unshift({ name, samples, date });
        if (T.recentFiles.length > 20) T.recentFiles.pop();
        renderFileList();
    }
    function renderFileList() {
        const ul = $('test-file-list');
        if (T.recentFiles.length === 0) {
            ul.innerHTML = '<li class="test-file-empty">none yet</li>';
            return;
        }
        ul.innerHTML = T.recentFiles.map(f =>
            `<li><span class="fname" title="${f.name}">${f.name}</span>` +
            `<span class="fmeta">${f.samples} · ${pad(f.date.getHours(),2)}:${pad(f.date.getMinutes(),2)}</span></li>`
        ).join('');
    }

    // ---- Manual entry ----
    async function saveManualEntry() {
        const testId = $('test-manual-id').value;
        const target = $('test-manual-target').value;
        const measured = $('test-manual-measured').value;
        const note = $('test-manual-note').value;
        if (target === '' && measured === '') {
            $('test-manual-status').textContent = 'Need at least target or measured.';
            return;
        }
        const now = new Date();
        const error = (target !== '' && measured !== '') ? (parseFloat(measured) - parseFloat(target)) : '';
        const row = [
            now.toISOString(), testId, target, measured, error, fmt(note)
        ].join(',');
        await appendManualCsv(row);
        $('test-manual-status').textContent = '✓ Saved ' + testId + ' @ ' + pad(now.getHours(),2)+':'+pad(now.getMinutes(),2)+':'+pad(now.getSeconds(),2);
        $('test-manual-target').value = '';
        $('test-manual-measured').value = '';
        $('test-manual-note').value = '';
    }

    async function appendManualCsv(rowLine) {
        const filename = 'manual_log.csv';
        const header = 'timestamp,test_id,target,measured,error,note\n';
        if (T.dirHandle) {
            try {
                const fh = await T.dirHandle.getFileHandle(filename, { create: true });
                const file = await fh.getFile();
                const existing = file.size > 0 ? await file.text() : '';
                const ws = await fh.createWritable();
                if (!existing) await ws.write(header);
                else await ws.write(existing);
                await ws.write(rowLine + '\n');
                await ws.close();
                return;
            } catch (e) {
                dlog('Manual append failed (' + e.message + ') — downloading single-row CSV.');
            }
        }
        const blob = new Blob([header + rowLine + '\n'], { type: 'text/csv' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'manual_' + Date.now() + '.csv';
        document.body.appendChild(a); a.click(); a.remove();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    // ---- Wire up UI ----
    function init() {
        $('btn-test-folder').addEventListener('click', pickFolder);
        $('btn-test-stop').addEventListener('click', stopRecording);

        document.querySelectorAll('.test-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const id = btn.dataset.test;
                const def = TESTS[id];
                if (!def) return;
                const params = [];
                for (const p of def.prompts) {
                    const v = prompt(def.label + ' — ' + p);
                    if (v === null) return; // cancelled
                    params.push(v);
                }
                const cmd = def.cmd(params);
                startRecording(id, params, cmd);
            });
        });

        $('btn-test-free').addEventListener('click', () => {
            const tag = $('test-free-tag').value.trim() || 'FREE';
            startRecording(tag, [], null);
        });

        $('btn-test-manual-save').addEventListener('click', saveManualEntry);

        renderFileList();
        dlog('Test Suite ready. Pick a folder for auto-save.');
    }

    // Expose hook for app.js
    window.TestSuite = { onPacket };

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', init);
    } else {
        init();
    }
})();
