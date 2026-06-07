// --- State ---
const state = {
    connected: false,
    currentPos: 0,
    targetPos: 0,
    firmwareTarget: 0,
    velSetpoint: 0,
    accSetpoint: 0,
    jogMode: 'COARSE',
    vel: 0,
    acc: 0,
    pwm: 0,
    mode: 'STOPPED',
    sysMode: 'BASE',
    fault: 'NONE',
    joystick: false,
    modbus: 'IDLE',
    ghost: false,
    prox: false,
    estop: false,
    gripper_ud: 0,
    gripper_co: 0,
    override: false,
    waypoints: [],
    seqActive: false,
    seqLoop: false,
    currentWaypointIdx: -1,

    // Kalman filter telemetry
    kfEnabled: false,
    kfTheta: 0,         // deg
    kfOmega: 0,         // RPM
    kfTau: 0,           // N·m
    kfIa: 0,            // A
    kfInnov: 0,
    kfP00: 0, kfP11: 0, kfP22: 0, kfP33: 0,
    kfSanityTheta: 0,   // deg, open-loop model
    kfSanityOmega: 0,   // RPM
    kfSanityShow: false,
};

let tuningSynced = false;
// Hoisted because renderPosLoopBtn() (called during script eval) references it.
let tuningMode = false;

// --- Charts & Visualizer ---
const visualizer = new RobotVisualizer('robot-visualizer');
const gripperVisualizer = new GripperVisualizer('gripper-visualizer');
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
/* Constructor: (id, color, liveMin, liveMax, tuneMin, tuneMax, absInTuning) */
const chartPos = new TelemetryChart('chart-pos', 'cyan', -360, 360, 0, 400, true);
const chartVel = new TelemetryChart('chart-vel', 'magenta', -100, 100, 0, 100, false);
const chartAcc = new TelemetryChart('chart-acc', 'yellow', -200, 200);

// --- UI Elements ---
const connectBtn = document.getElementById('connect-btn');
const estopBtn = document.getElementById('estop-btn');
const statusIndicator = document.getElementById('serial-status');
const logContent = document.getElementById('log-content');
const statMode = document.getElementById('stat-mode');
const statFault = document.getElementById('stat-fault');
const statJoy = document.getElementById('stat-joy');
const statModbus = document.getElementById('stat-modbus');
const statSysmode = document.getElementById('stat-sysmode');
const statJogmode = document.getElementById('stat-jogmode');
const ioProx = document.getElementById('io-prox');
const ioEstop = document.getElementById('io-estop');
const inputTarget = document.getElementById('input-target');
const rangeTarget = document.getElementById('range-target');
const btnGripperUD = document.getElementById('btn-gripper-ud');
const btnGripperCO = document.getElementById('btn-gripper-co');
const btnOverride = document.getElementById('btn-override');

// --- WebSocket ---
let socket = null;

async function connectSerial() {
    socket = new WebSocket('ws://localhost:8766');
    socket.onopen = () => {
        state.connected = true;
        updateUI();
        log("Connected to WebSocket bridge.");
    };
    socket.onmessage = (event) => {
        const packets = event.data.split('\n');
        packets.forEach(p => {
            if (p.trim()) processPacket(p.trim());
        });
    };
    socket.onclose = () => {
        state.connected = false;
        updateUI();
        log("Disconnected from WebSocket.", "error");
    };
    socket.onerror = (error) => {
        log("WebSocket error", "error");
    };
}

async function disconnectSerial() {
    if (socket) {
        socket.close();
        socket = null;
    }
}

let _dbgPacketCount = 0;

function processPacket(packet) {
    if (_dbgPacketCount < 5) {
        log("RAW PKT: " + packet);
        _dbgPacketCount++;
    }
    
    // Strip leading '$' if present, and trailing '*' (legacy framing)
    if (packet.startsWith('$')) packet = packet.substring(1);
    if (packet.endsWith('*')) packet = packet.substring(0, packet.length - 1);
    packet = packet.trim();
    if (!packet) return;
    
    // The new firmware sends CSV lines instead of Key:Value pairs.
    // e.g. $T,tick,pos*10,vel*10,acc*10,pwm*10
    // e.g. $ST,fsm,run_mode,fault
    const parts = packet.split(',');
    if (parts.length === 0) return;
    
    if (parts[0] === 'T') {
        if (parts.length >= 6) {
            state.currentPos = (parseFloat(parts[2]) || 0) / 10.0;  // degrees
            // Firmware sends vel/acc in rad/s × 10; convert to deg/s for dashboard
            // Firmware sends vel/acc in rad/s × 10; convert to RPM for dashboard
            const RAD2RPM = 60.0 / (2.0 * Math.PI);
            state.vel = ((parseFloat(parts[3]) || 0) / 10.0) * RAD2RPM;  // RPM
            state.acc = ((parseFloat(parts[4]) || 0) / 10.0) * RAD2RPM;  // RPM/s²
            state.pwm = (parseFloat(parts[5]) || 0) / 10.0;
            if (parts.length >= 7) {
                state.velSetpoint = ((parseFloat(parts[6]) || 0) / 10.0) * RAD2RPM;
            }
        }
    } else if (parts[0] === 'ST') {
        if (parts.length >= 4) {
            const fsm = parseInt(parts[1]) || 0;
            const run_mode = parseInt(parts[2]) || 0;
            const fault = parseInt(parts[3]) || 0;
            
            state.mode = decodeMode(fsm);
            state.fault = decodeFault(fault);
        }
    } else if (parts[0] === 'CTRL') {
        if (parts.length >= 13) {
            state.firmwareTarget = (parseFloat(parts[2]) || 0) / 10.0;
            // shp is parts[4], p_act is parts[5], v_cmd is parts[7]
            state.velSetpoint = (parseFloat(parts[7]) || 0) / 10.0;
            state.kfTheta = (parseFloat(parts[5]) || 0) / 100.0;  // kal_pos_x100
            state.kfOmega = (parseFloat(parts[8]) || 0) / 10.0;   // kal_vel_x10
            state.posIntegral = (parseFloat(parts[11]) || 0) / 10.0;
            state.spdIntegral = (parseFloat(parts[12]) || 0) / 10.0;
        }
    } else if (parts[0] === 'KF') {
        if (parts.length >= 6) {
            state.kfTheta = (parseFloat(parts[1]) || 0) / 100.0;
            state.kfOmega = (parseFloat(parts[2]) || 0) / 10.0;
            state.kfTau = (parseFloat(parts[3]) || 0) / 1000.0;
            state.kfIa = (parseFloat(parts[4]) || 0) / 1000.0;
            state.kfInnov = (parseFloat(parts[5]) || 0) / 1000.0;
            if (parts.length >= 8) {
                state.kfSanityTheta = parseFloat(parts[6]) || 0;
                state.kfSanityOmega = parseFloat(parts[7]) || 0;
            }
            state.kfP00 = parseFloat(parts[6]) || 0;
            state.kfP11 = parseFloat(parts[7]) || 0;
            state.kfP22 = parseFloat(parts[8]) || 0;
            state.kfEnabled = true;
        }
    } else if (parts[0] === 'WCET') {
        // $WCET,enc,kf,traj,ff,dist,pospid,velpid,motor,total
        if (parts.length >= 10) {
            state.wcet = {
                encoder: parseInt(parts[1]) || 0,
                kalman: parseInt(parts[2]) || 0,
                trajectory: parseInt(parts[3]) || 0,
                feedforward: parseInt(parts[4]) || 0,
                dist_comp: parseInt(parts[5]) || 0,
                pos_pid: parseInt(parts[6]) || 0,
                vel_pid: parseInt(parts[7]) || 0,
                motor: parseInt(parts[8]) || 0,
                total: parseInt(parts[9]) || 0,
            };
        }
    } else if (parts[0] === 'GAINS') {
        // $GAINS,loop,kp*1000,ki*1000,kd*1000
        if (parts.length >= 5) {
            const loop = parseInt(parts[1]) || 0;
            const kp = (parseInt(parts[2]) || 0) / 1000.0;
            const ki = (parseInt(parts[3]) || 0) / 1000.0;
            const kd = (parseInt(parts[4]) || 0) / 1000.0;
            if (loop === 0) { // Velocity
                syncTuning('input-speed-kp', kp);
                syncTuning('input-speed-ki', ki);
                syncTuning('input-speed-kd', kd);
            } else { // Position
                syncTuning('input-pos-kp', kp);
                syncTuning('input-pos-ki', ki);
                syncTuning('input-pos-kd', kd);
            }
        }
    } else if (parts[0] === 'DASH' || parts[0] === 'HOM' || parts[0] === 'AUTO') {
        // Informational packets — log but don't parse further
        log("FW: " + packet);
    } else {
        // Fallback for legacy key:val format if any still exist
        parts.forEach(pair => {
            const idx = pair.indexOf(':');
            if (idx === -1) return;
            const key = pair.substring(0, idx).trim();
            const val = pair.substring(idx + 1).trim();
            const num = parseFloat(val);
            const safeNum = isNaN(num) ? 0 : num;

            switch (key) {
                case 'POS': state.currentPos = safeNum; break;
                case 'VEL': state.vel = safeNum; break;
                case 'ACC': state.acc = safeNum; break;
                case 'TAR': state.firmwareTarget = safeNum; break;
                case 'VSET': state.velSetpoint = safeNum; break;
                case 'ASET': state.accSetpoint = safeNum; break;
                case 'PWM': state.pwm = safeNum; break;
                case 'MODE': state.mode = decodeMode(val); break;
                case 'SYSM': state.sysMode = val === '1' ? 'JOYSTICK' : 'BASE'; break;
                case 'JOGM': state.jogMode = val === '1' ? 'FINE' : 'COARSE'; break;
                case 'JOY': state.joystick = val === '1'; break;
                case 'ESTOP': state.estop = val === '1'; break;
                case 'FAULT': state.fault = decodeFault(val); break;
                case 'PROX': state.prox = val !== '1'; break;
                case 'GHOST': state.ghost = val === '1'; break;
                case 'GUD': state.gripper_ud = val === '1'; break;
                case 'GCO': state.gripper_co = val === '1'; break;
                case 'KFEN': state.kfEnabled = val === '1'; break;
                case 'KTH': state.kfTheta = safeNum; break;
                case 'KOM': state.kfOmega = safeNum; break;
                case 'KTL': state.kfTau = safeNum; break;
                case 'KIA': state.kfIa = safeNum; break;
                case 'KIV': state.kfInnov = safeNum; break;
                case 'KP00': state.kfP00 = safeNum; break;
                case 'KP11': state.kfP11 = safeNum; break;
                case 'KP22': state.kfP22 = safeNum; break;
                case 'KP33': state.kfP33 = safeNum; break;
                case 'KSTH': state.kfSanityTheta = safeNum; break;
                case 'KSOM': state.kfSanityOmega = safeNum; break;
                case 'PKP': syncTuning('input-pos-kp', val); break;
                case 'PKI': syncTuning('input-pos-ki', val); break;
                case 'PKD': syncTuning('input-pos-kd', val); break;
                case 'SKP': syncTuning('input-speed-kp', val); break;
                case 'SKI': syncTuning('input-speed-ki', val); break;
                case 'SKD': syncTuning('input-speed-kd', val); break;
                case 'KVFF': syncTuning('input-k-vff', val); break;
                case 'KAFF': syncTuning('input-k-aff', val); break;
                case 'JMAX': syncTuning('input-max-jerk-rpm', val); /* legacy RPM/s² field if present */
                    /* Also convert to rad/s³ for the SI input box */
                    syncTuning('input-jmax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
                case 'VMAX': syncTuning('input-vmax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
                case 'AMAX': syncTuning('input-amax-rad', (parseFloat(val) || 0) * (2 * Math.PI / 60)); break;
                case 'STEPC': syncTuning('input-step-coarse', val); break;
                case 'STEPF': syncTuning('input-step-fine', val); break;
                case 'JOGF': syncTuning('input-jog-fine', val); break;
                case 'HOMES': syncTuning('input-home-speed', val); break;
                case 'MINP': syncTuning('input-min-pwm', val); break;
                case 'PLOOP': {
                    // Ignore for 2 s after the user clicked, otherwise a stale
                    // in-flight slow telemetry packet would overwrite the click.
                    if (Date.now() - posLoopUserClickedAt < 2000) break;
                    const sw = document.getElementById('switch-outer-loop');
                    if (sw) sw.checked = (val === '1');
                    break;
                }
            }
        });
    }

    lockTuningAfterSync();
    updateUI();
    visualizer.update(state.currentPos, state.firmwareTarget);
    gripperVisualizer.update(!!state.gripper_ud, !state.gripper_co);
    chartPos.addData(state.currentPos);
    chartPos.addTarget(state.firmwareTarget);
    chartVel.addData(state.vel);
    chartVel.addTarget(state.velSetpoint);
    chartAcc.addData(state.acc);
    chartAcc.addTarget(state.accSetpoint);

    // KF overlays (always pushed; rendering on each chart is unconditional
    // because the chart class hides empty arrays). When KF is disabled the
    // estimate is just the open-loop integration with corrections.
    chartPos.addEstimate(state.kfTheta);
    chartVel.addEstimate(state.kfOmega);
    if (state.kfSanityShow) {
        chartPos.addSanity(state.kfSanityTheta);
        chartVel.addSanity(state.kfSanityOmega);
    }
    updateKalmanCard();
    checkGhostStart();
    tuningTick();
    if (window.TestSuite) window.TestSuite.onPacket(state, packet);
}

function sendWsMessage(obj) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(obj));
    } else {
        console.warn("WebSocket not connected - command dropped", obj);
    }
}

async function sendCommand(cmd) {
    if (!state.connected || !socket) return;
    log("Sent (Internal): " + cmd);
    
    if (cmd.startsWith("SET:")) {
        const [key, valStr] = cmd.substring(4).split("=");
        const val = parseFloat(valStr);
        if (isNaN(val)) return;
        
        switch(key) {
            case "TARGET": sendWsMessage({ action: 'cmd', cmd: `LAB P2P ${val.toFixed(1)}` }); break;
            case "SPEED_KP": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 0 }); sendWsMessage({ action: 'write_reg', reg: 0x38, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "SPEED_KI": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 0 }); sendWsMessage({ action: 'write_reg', reg: 0x39, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "SPEED_KD": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 0 }); sendWsMessage({ action: 'write_reg', reg: 0x3A, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "POS_KP": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 1 }); sendWsMessage({ action: 'write_reg', reg: 0x38, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "POS_KI": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 1 }); sendWsMessage({ action: 'write_reg', reg: 0x39, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "POS_KD": sendWsMessage({ action: 'write_reg', reg: 0x36, val: 1 }); sendWsMessage({ action: 'write_reg', reg: 0x3A, val: Math.round(val*1000) }); sendWsMessage({ action: 'write_reg', reg: 0x33, val: 2 }); break;
            case "K_VFF": sendWsMessage({ action: 'cmd', cmd: `LAB FF_VEL ${val.toFixed(3)}`}); break;
            case "K_AFF": sendWsMessage({ action: 'cmd', cmd: `LAB FF_ACC ${val.toFixed(3)}`}); break;
            case "V_MAX_RAD": sendWsMessage({ action: 'cmd', cmd: `LAB SC_VMAX ${val.toFixed(3)}`}); break;
            case "A_MAX_RAD": sendWsMessage({ action: 'cmd', cmd: `LAB SC_AMAX ${val.toFixed(3)}`}); break;
            case "J_MAX_RAD": sendWsMessage({ action: 'cmd', cmd: `LAB SC_JMAX ${val.toFixed(0)}`}); break;
            case "MIN_PWM": sendWsMessage({ action: 'cmd', cmd: `LAB SC_MINPWM ${val.toFixed(2)}`}); break;
            case "HOME_SPEED": sendWsMessage({ action: 'cmd', cmd: `LAB HOME_SPD ${val.toFixed(1)}`}); break;
            case "JOG_MODE": sendWsMessage({ action: 'write_reg', reg: 0x01, val: 2 }); break; // 2 = Jog mode
            case "KF_EN": sendWsMessage({ action: 'cmd', cmd: `LAB KF_EN ${val}`}); break;
            case "KF_RESET": sendWsMessage({ action: 'cmd', cmd: `LAB KF_RESET 1`}); break;
            case "ZV_WN": sendWsMessage({ action: 'cmd', cmd: `LAB ZV_WN ${val.toFixed(2)}`}); break;
            case "ZV_ZETA": sendWsMessage({ action: 'cmd', cmd: `LAB ZV_ZETA ${val.toFixed(3)}`}); break;
            case "ZVD_EN": sendWsMessage({ action: 'cmd', cmd: `LAB ZVD_EN ${val}`}); break;
            case "KF_Q_THETA": sendWsMessage({ action: 'cmd', cmd: `LAB KF_Q_TH ${val.toExponential(3)}`}); break;
            case "KF_Q_OMEGA": sendWsMessage({ action: 'cmd', cmd: `LAB KF_Q_W ${val.toExponential(3)}`}); break;
            case "KF_Q_TAU": sendWsMessage({ action: 'cmd', cmd: `LAB KF_Q_TAU ${val.toExponential(3)}`}); break;
            case "KF_Q_I": sendWsMessage({ action: 'cmd', cmd: `LAB KF_Q_I ${val.toExponential(3)}`}); break;
            case "KF_R": sendWsMessage({ action: 'cmd', cmd: `LAB KF_R ${val.toExponential(3)}`}); break;
            case "SAFE_STALL": sendWsMessage({ action: 'cmd', cmd: `LAB SAFE_STALL ${val}`}); break;
            case "SAFE_ENCODER": sendWsMessage({ action: 'cmd', cmd: `LAB SAFE_ENC ${val}`}); break;
            case "SAFE_OVERROT": sendWsMessage({ action: 'cmd', cmd: `LAB SAFE_OVER ${val}`}); break;
            case "SAFE_JOY": sendWsMessage({ action: 'cmd', cmd: `LAB SAFE_JOY ${val}`}); break;
            case "JOG_FINE": sendWsMessage({ action: 'cmd', cmd: `LAB JOGF ${val.toFixed(1)}`}); break;
            case "STEP_COARSE": sendWsMessage({ action: 'cmd', cmd: `LAB STEPC ${val.toFixed(1)}`}); break;
            case "STEP_FINE": sendWsMessage({ action: 'cmd', cmd: `LAB STEPF ${val.toFixed(2)}`}); break;
            default:
                console.warn("Unknown SET command:", key);
                break;
        }
    } else if (cmd.startsWith("CMD:")) {
        const cmdName = cmd.substring(4);
        if (cmdName === "HOME") {
            sendWsMessage({ action: 'write_reg', reg: 0x01, val: 1 });
        } else if (cmdName === "ESTOP=1") {
            sendWsMessage({ action: 'write_reg', reg: 0x25, val: 1 });
        } else if (cmdName === "CLEAR") {
            sendWsMessage({ action: 'write_reg', reg: 0x25, val: 0 });
        } else if (cmdName === "GRIP_UP") {
            sendWsMessage({ action: 'write_reg', reg: 0x02, val: 0 });
        } else if (cmdName === "GRIP_DN") {
            sendWsMessage({ action: 'write_reg', reg: 0x02, val: 1 });
        } else if (cmdName === "GRIP_OPEN") {
            sendWsMessage({ action: 'write_reg', reg: 0x02, val: 2 });
        } else if (cmdName === "GRIP_CLOSE") {
            sendWsMessage({ action: 'write_reg', reg: 0x02, val: 4 });
        }
    }
}

// --- UI ---
function updateUI() {
    statusIndicator.innerText = state.connected ? "Connected" : "Disconnected";
    statusIndicator.className = "status-indicator " + (state.connected ? "connected" : "disconnected");
    connectBtn.innerText = state.connected ? "Disconnect" : "Connect Robot";

    statMode.innerText = state.mode;
    statFault.innerText = state.fault;
    statFault.className = "value " + (state.fault === 'NONE' ? "ok" : "error");
    statJoy.innerText = state.joystick ? "CONNECTED" : "OFFLINE";
    statJoy.className = "value " + (state.joystick ? "ok" : "");
    statModbus.innerText = state.modbus;
    statSysmode.innerText = state.sysMode;
    statSysmode.className = "value " + (state.sysMode === 'JOYSTICK' ? "ok" : "");
    statJogmode.innerText = state.jogMode;
    statJogmode.className = "value " + (state.jogMode === 'FINE' ? "ok" : "");

    const bjm = document.getElementById('btn-jog-mode');
    if (bjm) {
        bjm.innerText = 'Jog: ' + state.jogMode;
        bjm.className = 'toggle-btn ' + (state.jogMode === 'FINE' ? 'active' : '');
    }

    document.getElementById('stat-pwm').innerText = (isNaN(state.pwm) ? '0.0' : state.pwm.toFixed(1)) + "%";
    ioProx.className = "io-item " + (state.prox ? "active" : "");
    ioEstop.className = "io-item " + (state.estop ? "active" : "");

    btnGripperUD.innerText = "Gripper: " + (state.gripper_ud ? "DOWN" : "UP");
    btnGripperUD.className = "toggle-btn " + (state.gripper_ud ? "active" : "active-green");
    btnGripperCO.innerText = "Claw: " + (state.gripper_co ? "CLOSED" : "OPEN");
    btnGripperCO.className = "toggle-btn " + (state.gripper_co ? "active" : "active-green");
    btnOverride.innerText = "Override: " + (state.override ? "ON" : "OFF");
    btnOverride.className = "toggle-btn warning " + (state.override ? "active" : "");
    estopBtn.innerText = state.estop ? "CLEAR FAULT / RESUME" : "EMERGENCY STOP";
    estopBtn.className = state.estop ? "danger-btn active" : "danger-btn";

    const btnSysMode = document.getElementById('btn-sys-mode');
    btnSysMode.innerText = "Mode: " + state.sysMode;
    btnSysMode.className = "toggle-btn " + (state.sysMode === 'JOYSTICK' ? "active" : "");

}

function log(msg, type = "info") {
    const time = new Date().toLocaleTimeString([], { hour12: false });
    const div = document.createElement('div');
    div.innerHTML = `<span style="color:#555">[${time}]</span> ${msg}`;
    if (type === "error") div.style.color = "#ff3131";
    logContent.prepend(div);
}

let _tuningReceivedFromFirmware = false;

function syncTuning(id, val) {
    const el = document.getElementById(id);
    if (!el || tuningSynced) return;
    const num = parseFloat(val);
    if (!isNaN(num)) el.value = num;
    _tuningReceivedFromFirmware = true;
}

function lockTuningAfterSync() {
    if (_tuningReceivedFromFirmware) tuningSynced = true;
}

function decodeMode(val) {
    return ['STOPPED', 'SPEED', 'POSITION', 'AUTOTUNE_P', 'AUTOTUNE_S', 'TEST', 'GHOST', 'HOMING'][parseInt(val)] || 'UNKNOWN';
}

function decodeFault(val) {
    const bits = parseInt(val);
    if (bits === 0) return 'NONE';
    const f = [];
    if (bits & 0x001) f.push('STALL');
    if (bits & 0x002) f.push('ENCODER');
    if (bits & 0x004) f.push('JOY_LOST');
    if (bits & 0x008) f.push('OVER_ROT');
    if (bits & 0x010) f.push('ESTOP_HW');
    if (bits & 0x020) f.push('PROX_LOST');
    if (bits & 0x040) f.push('ESTOP_JOY');
    if (bits & 0x080) f.push('ESTOP_DASH');
    if (bits & 0x100) f.push('ESTOP_MBUS');
    return f.join(' | ');
}

// --- Event Listeners ---
connectBtn.addEventListener('click', () => {
    if (state.connected) disconnectSerial();
    else connectSerial();
});

estopBtn.addEventListener('click', () => {
    if (state.estop) {
        sendCommand("CMD:CLEAR");
        state.estop = false;
        state.fault = 'NONE';
    } else {
        sendCommand("CMD:ESTOP=1");
        state.estop = true;
        state.vel = 0;
        state.acc = 0;
        chartVel.history = [];
        chartAcc.history = [];
        chartVel.draw();
        chartAcc.draw();
    }
    updateUI();
});

inputTarget.addEventListener('input', e => {
    state.targetPos = parseFloat(e.target.value);
    rangeTarget.value = state.targetPos;
});

rangeTarget.addEventListener('input', e => {
    state.targetPos = parseFloat(e.target.value);
    inputTarget.value = state.targetPos;
});

document.getElementById('btn-negate-target').addEventListener('click', () => {
    const v = parseFloat(inputTarget.value) || 0;
    const neg = -v;
    inputTarget.value = neg;
    rangeTarget.value = neg;
    state.targetPos = neg;
});

document.getElementById('btn-set-target').addEventListener('click', () => {
    const target = parseFloat(inputTarget.value);
    if (!isNaN(target)) {
        sendCommand(`SET:TARGET=${target}`);
        if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(target); setMetricsStatus('ARMED (Move) — Waiting for motor motion...', ''); }
    }
});

btnGripperUD.addEventListener('click', () => {
    state.gripper_ud = 1 - state.gripper_ud;
    sendCommand(state.gripper_ud ? "CMD:GRIP_DN" : "CMD:GRIP_UP");
    updateUI();
});

btnGripperCO.addEventListener('click', () => {
    state.gripper_co = 1 - state.gripper_co;
    sendCommand(state.gripper_co ? "CMD:GRIP_CLOSE" : "CMD:GRIP_OPEN");
    updateUI();
});

document.getElementById('btn-sys-mode').addEventListener('click', () => {
    sendCommand('CMD:TOGGLE_MODE');
    log('Sent TOGGLE_MODE — LPUART1 baud will switch. If going to BASE, dashboard will disconnect.');
});

// Position loop enable/disable — bypass outer loop to tune velocity loop alone
const btnPosLoop = document.getElementById('btn-pos-loop');
let posLoopEnabled = true;
let posLoopUserClickedAt = 0;   // timestamp of last user click on the Loop button
const POS_INPUT_IDS = ['input-pos-kp', 'input-pos-ki', 'input-pos-kd'];
let sineActive = false;
let sineCapturing = false;
let sineStartMs = 0;
let prevTuningModeBeforeLoopOff = null;  // remembers the user's mode so we can restore it

// Snapshot the default tuning-axis scaling so we can swap it while the
// position loop is OFF (sine wave needs a symmetric, signed Y range).
const TUNE_SCALE_DEFAULT = {
    pos: { min: chartPos.tuningMinVal, max: chartPos.tuningMaxVal, abs: chartPos.absInTuning },
    vel: { min: chartVel.tuningMinVal, max: chartVel.tuningMaxVal, abs: chartVel.absInTuning },
    acc: { min: chartAcc.tuningMinVal, max: chartAcc.tuningMaxVal, abs: chartAcc.absInTuning },
};
function applyTuningChartScale(loopOff) {
    if (loopOff) {
        chartPos.tuningMinVal = -360; chartPos.tuningMaxVal = 360; chartPos.absInTuning = false;
        chartVel.tuningMinVal = -60; chartVel.tuningMaxVal = 60; chartVel.absInTuning = false;
        // acc has no tuning override -> nothing to do
    } else {
        chartPos.tuningMinVal = TUNE_SCALE_DEFAULT.pos.min;
        chartPos.tuningMaxVal = TUNE_SCALE_DEFAULT.pos.max;
        chartPos.absInTuning = TUNE_SCALE_DEFAULT.pos.abs;
        chartVel.tuningMinVal = TUNE_SCALE_DEFAULT.vel.min;
        chartVel.tuningMaxVal = TUNE_SCALE_DEFAULT.vel.max;
        chartVel.absInTuning = TUNE_SCALE_DEFAULT.vel.abs;
    }
    chartPos.draw(); chartVel.draw(); chartAcc.draw();
}
function renderPosLoopBtn() {
    btnPosLoop.textContent = posLoopEnabled ? 'Loop: ON' : 'Loop: OFF';
    btnPosLoop.classList.toggle('active', posLoopEnabled);
    btnPosLoop.classList.toggle('danger', !posLoopEnabled);

    // Gray out + lock the position-PID inputs when the outer loop is bypassed
    const section = btnPosLoop.closest('.tuning-section');
    if (section) section.classList.toggle('disabled', !posLoopEnabled);
    POS_INPUT_IDS.forEach(id => {
        const el = document.getElementById(id);
        if (el) el.disabled = !posLoopEnabled;
    });

    // Swap Manual Override target row <-> sine wave generator
    const posGrp = document.getElementById('target-pos-group');
    const sineGrp = document.getElementById('sine-gen-group');
    if (posGrp && sineGrp) {
        posGrp.style.display = posLoopEnabled ? '' : 'none';
        sineGrp.style.display = posLoopEnabled ? 'none' : '';
    }
    // Hide Gripper/Claw/Mode/Jog/Override during tuning — keep only Fine/Go Home
    document.querySelectorAll('.manual-extra').forEach(el => {
        el.style.display = posLoopEnabled ? '' : 'none';
    });

    // Loop OFF: switch the telemetry view to Tuning so the preview/run lives
    // in the right chart mode. Loop ON: restore the user's prior mode.
    if (!posLoopEnabled) {
        if (prevTuningModeBeforeLoopOff === null) prevTuningModeBeforeLoopOff = tuningMode;
        if (!tuningMode) setTuningMode(true);
    } else if (prevTuningModeBeforeLoopOff !== null) {
        if (tuningMode !== prevTuningModeBeforeLoopOff) setTuningMode(prevTuningModeBeforeLoopOff);
        prevTuningModeBeforeLoopOff = null;
    }

    // Tuning Y-axis: when loop is OFF the signal can go negative (sine wave),
    // so use symmetric ranges and disable abs-folding on the pos chart.
    applyTuningChartScale(!posLoopEnabled);

    // Sine preview on velocity chart (only when loop OFF and sine not yet started)
    if (!posLoopEnabled && !sineActive) {
        refreshSinePreview();
    } else {
        if (chartVel.clearPreviewSine) chartVel.clearPreviewSine();
    }

    // Killing the loop also stops any running sine — make sure firmware agrees
    if (posLoopEnabled && sineActive) stopSine();

    // Loop OFF → show Kalman card in place of Step Response Metrics.
    // Loop ON  → restore Step Response Metrics (only when in Tuning mode).
    const kf = document.querySelector('.kalman-card');
    const metrics = document.querySelector('.metrics-card');
    if (kf && metrics) {
        if (!posLoopEnabled) {
            kf.style.display = '';
            metrics.style.display = 'none';
        } else {
            kf.style.display = 'none';
            metrics.style.display = tuningMode ? '' : 'none';
        }
    }
}

function refreshSinePreview() {
    if (!chartVel.setPreviewSine) return;
    const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
    const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
    chartVel.setPreviewSine(amp, freq);
}

function stopSine() {
    sineActive = false;
    sendCommand('SET:SINE_EN=0');
    // End the capture and lock the trace in tuning-run history
    if (sineCapturing) {
        sineCapturing = false;
        chartVel.finalizeRun(null, null);
    }
    chartPos.scrollWindowSec = null;
    chartVel.scrollWindowSec = null;
    chartAcc.scrollWindowSec = null;
    const btn = document.getElementById('btn-sine-toggle');
    if (btn) { btn.textContent = 'Start Sine'; btn.classList.remove('danger'); }
    // Restore preview overlay (loop is still OFF)
    if (!posLoopEnabled) refreshSinePreview();
}
function startSine() {
    const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
    const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
    sendCommand(`SET:SINE_AMP=${amp}`);
    sendCommand(`SET:SINE_FREQ=${freq}`);
    sendCommand('SET:SINE_EN=1');
    sineActive = true;
    if (chartVel.clearPreviewSine) chartVel.clearPreviewSine();  // captured trace takes over
    // Begin a tuning-mode capture window on the velocity chart.
    // Use a scrolling 10-second window so the wave keeps the same
    // horizontal density as the run grows.
    sineStartMs = Date.now();
    chartVel.startRun(0);
    chartPos.scrollWindowSec = 10;
    chartVel.scrollWindowSec = 10;
    chartAcc.scrollWindowSec = 10;
    sineCapturing = true;
    const btn = document.getElementById('btn-sine-toggle');
    if (btn) { btn.textContent = 'Stop Sine'; btn.classList.add('danger'); }
    log(`Sine generator ON — ${amp} RPM @ ${freq} Hz`);
}
document.getElementById('btn-sine-toggle').addEventListener('click', () => {
    sineActive ? stopSine() : startSine();
});
// Live-update amp/freq + refresh preview as you type
['input-sine-amp', 'input-sine-freq'].forEach(id => {
    const el = document.getElementById(id);
    el.addEventListener('input', () => { if (!sineActive && !posLoopEnabled) refreshSinePreview(); });
    el.addEventListener('change', () => {
        if (sineActive) {
            const k = id === 'input-sine-amp' ? 'SINE_AMP' : 'SINE_FREQ';
            sendCommand(`SET:${k}=${parseFloat(el.value) || 0}`);
        }
    });
});
btnPosLoop.addEventListener('click', () => {
    posLoopEnabled = !posLoopEnabled;
    posLoopUserClickedAt = Date.now();
    sendCommand(`SET:POS_LOOP=${posLoopEnabled ? 1 : 0}`);
    renderPosLoopBtn();
    log(`Position loop ${posLoopEnabled ? 'ENABLED' : 'BYPASSED — velocity loop tuning mode'}`);
});
renderPosLoopBtn();

const btnZvd = document.getElementById('btn-zvd');
let zvdEnabled = true;

function renderZvdBtn() {
    if (!btnZvd) return;
    btnZvd.textContent = zvdEnabled ? 'ZVD: ON' : 'ZVD: OFF';
    btnZvd.classList.toggle('active', zvdEnabled);
    btnZvd.classList.toggle('danger', !zvdEnabled);
}

if (btnZvd) {
    btnZvd.addEventListener('click', () => {
        zvdEnabled = !zvdEnabled;
        sendCommand(`SET:ZVD_EN=${zvdEnabled ? 1 : 0}`);
        renderZvdBtn();
        log(`ZVD Input Shaper ${zvdEnabled ? 'ENABLED' : 'DISABLED'}`);
    });
    renderZvdBtn();
}

document.getElementById('btn-jog-mode').addEventListener('click', () => {
    const newJog = state.jogMode === 'COARSE' ? 1 : 0;
    sendCommand(`SET:JOG_MODE=${newJog}`);
    state.jogMode = newJog === 1 ? 'FINE' : 'COARSE';
    updateUI();
});

btnOverride.addEventListener('click', () => {
    state.override = !state.override;
    sendCommand(`SET:OVERRIDE=${state.override ? 1 : 0}`);
    updateUI();
});

// --- Fault Modal ---
const FAULT_INFO = {
    /* Automatic faults (gated by safety_config) */
    'STALL': { name: 'Motor Stalled', desc: 'PWM high but rotor not moving for 2 s. Source: automatic safety monitor. Check obstruction or wiring.' },
    'ENCODER': { name: 'Encoder Error', desc: 'Encoder signal lost or phase inverted. Source: automatic safety monitor. Check encoder cable.' },
    'JOY_LOST': { name: 'Joystick Lost', desc: 'ESP32 joystick disconnected. Source: automatic safety monitor (Joystick Check).' },
    'OVER_ROT': { name: 'Over-Rotation', desc: 'Exceeded ±720° from home. Source: soft-limit watchdog (wire-twist protection).' },
    /* User / external e-stop sources (informational; not gated by safety_config) */
    'ESTOP_HW': { name: 'E-Stop: Physical', desc: 'Hardware E-Stop button was pressed (GPIO EXTI). Release the button and press the physical Reset.' },
    'PROX_LOST': { name: 'Proximity Lost', desc: 'Proximity sensor reported open. Source: physical interlock.' },
    'ESTOP_JOY': { name: 'E-Stop: Joystick', desc: 'E-Stop triggered by the joystick safety button (P) or command (X). Source: ESP32 joystick.' },
    'ESTOP_DASH': { name: 'E-Stop: Dashboard', desc: 'EMERGENCY STOP button on the dashboard was clicked. Source: user via dashboard.' },
    'ESTOP_MBUS': { name: 'E-Stop: Modbus', desc: 'Modbus register 0x25 requested soft stop. Source: Base System / Modbus master.' },
};

function refreshFaultModal() {
    const list = document.getElementById('modal-fault-list');
    list.innerHTML = '';
    if (state.fault === 'NONE') {
        list.innerHTML = '<div style="color:var(--accent-green);">No active faults.</div>';
        return;
    }
    state.fault.split(' | ').forEach(f => {
        const info = FAULT_INFO[f] || { name: f, desc: 'Unknown fault.' };
        const div = document.createElement('div');
        div.className = 'fault-item';
        div.innerHTML = `<div class="fault-name">${info.name}</div><div class="fault-desc">${info.desc}</div>`;
        list.appendChild(div);
    });
}

/* Step Response Config modal */
function openMetricsConfig() {
    document.getElementById('cfg-vel-threshold').value = TUNE_VEL_THRESHOLD;
    document.getElementById('cfg-pos-settle').value = TUNE_POS_SETTLE_DEG;
    document.getElementById('cfg-vel-settle').value = TUNE_VEL_SETTLE_RPM;
    document.getElementById('cfg-settle-ms').value = TUNE_SETTLE_MS;
    document.getElementById('metrics-config-modal').classList.remove('hidden');
}
function saveMetricsConfig() {
    TUNE_VEL_THRESHOLD = parseFloat(document.getElementById('cfg-vel-threshold').value) || 3.0;
    TUNE_POS_SETTLE_DEG = parseFloat(document.getElementById('cfg-pos-settle').value) || 2.0;
    TUNE_VEL_SETTLE_RPM = parseFloat(document.getElementById('cfg-vel-settle').value) || 3.0;
    TUNE_SETTLE_MS = parseInt(document.getElementById('cfg-settle-ms').value) || 3000;
    localStorage.setItem('metricsCfg', JSON.stringify({
        velThresh: TUNE_VEL_THRESHOLD,
        posSettle: TUNE_POS_SETTLE_DEG,
        velSettle: TUNE_VEL_SETTLE_RPM,
        settleMs: TUNE_SETTLE_MS,
    }));
    document.getElementById('metrics-config-modal').classList.add('hidden');
    log(`Settle config saved: vel>${TUNE_VEL_THRESHOLD} pos<${TUNE_POS_SETTLE_DEG}° vel<${TUNE_VEL_SETTLE_RPM} hold=${TUNE_SETTLE_MS}ms`);
}
document.getElementById('btn-metrics-config').addEventListener('click', openMetricsConfig);
document.getElementById('btn-close-metrics-modal').addEventListener('click', () => {
    document.getElementById('metrics-config-modal').classList.add('hidden');
});
document.getElementById('btn-save-metrics-cfg').addEventListener('click', saveMetricsConfig);
document.getElementById('metrics-config-modal').addEventListener('click', e => {
    if (e.target.id === 'metrics-config-modal') e.target.classList.add('hidden');
});

document.getElementById('btn-fault-info').addEventListener('click', () => {
    refreshFaultModal();
    document.getElementById('fault-modal').classList.remove('hidden');
});
document.getElementById('btn-close-modal').addEventListener('click', () => {
    document.getElementById('fault-modal').classList.add('hidden');
});
document.getElementById('fault-modal').addEventListener('click', (e) => {
    if (e.target.id === 'fault-modal') e.target.classList.add('hidden');
});

const SAFETY_TOGGLES = {
    'chk-safe-stall': 'SAFE_STALL',
    'chk-safe-encoder': 'SAFE_ENCODER',
    'chk-safe-overrot': 'SAFE_OVERROT',
    'chk-safe-joy': 'SAFE_JOY',
};
Object.entries(SAFETY_TOGGLES).forEach(([id, key]) => {
    const el = document.getElementById(id);
    el.addEventListener('change', e => {
        sendCommand(`SET:${key}=${e.target.checked ? 1 : 0}`);
    });
    /* Push current checkbox state to firmware once we're connected — handles
     * the case where the user unchecked the box, reloaded the dashboard,
     * and the firmware still has the safety check enabled. */
    el.addEventListener('click', () => {
        /* fires after the toggle; redundant SET ensures sync after reload */
        sendCommand(`SET:${key}=${el.checked ? 1 : 0}`);
    });
});

document.getElementById('btn-fine-home').addEventListener('click', () => {
    sendCommand("CMD:HOME");
    log("Homing sequence triggered");
});

document.getElementById('btn-go-home').addEventListener('click', () => {
    sendCommand("SET:TARGET=0");
    log("Returning to home (0°)");
    if (tuningMode) { tuningState = 'IDLE'; tuningArmRun(0); setMetricsStatus('ARMED (Go Home) — Waiting for motor motion...', ''); }
});

document.getElementById('send-tuning-btn').addEventListener('click', () => {
    const params = {
        'SPEED_KP': 'input-speed-kp',
        'SPEED_KI': 'input-speed-ki',
        'SPEED_KD': 'input-speed-kd',
        'K_VFF': 'input-k-vff',
        'K_AFF': 'input-k-aff',
        /* SI-unit S-curve limits — firmware converts to RPM internally */
        'V_MAX_RAD': 'input-vmax-rad',
        'A_MAX_RAD': 'input-amax-rad',
        'POS_KP': 'input-pos-kp',
        'POS_KI': 'input-pos-ki',
        'POS_KD': 'input-pos-kd',
        'MIN_PWM': 'input-min-pwm',
        'HOME_SPEED': 'input-home-speed',
        'JOG_FINE': 'input-jog-fine',
        'STEP_COARSE': 'input-step-coarse',
        'STEP_FINE': 'input-step-fine',
        'ZV_WN': 'input-shaper-wn',
        'ZV_ZETA': 'input-shaper-zeta',
        'KF_Q_THETA': 'input-kf-q-theta',
        'KF_Q_OMEGA': 'input-kf-q-omega',
        'KF_Q_TAU': 'input-kf-q-tau',
        'KF_Q_I': 'input-kf-q-i',
        'KF_R': 'input-kf-r'
    };
    for (const [key, id] of Object.entries(params)) {
        sendCommand(`SET:${key}=${document.getElementById(id).value}`);
    }
    
    // Velocity Profile dropdown logic
    const velProfile = document.getElementById('select-vel-profile').value;
    if (velProfile === 'trap') {
        sendCommand('SET:J_MAX_RAD=99999'); // Massive jerk makes it a trapezoidal profile
    } else {
        sendCommand(`SET:J_MAX_RAD=${document.getElementById('input-jmax-rad').value}`);
    }
});

document.getElementById('select-vel-profile').addEventListener('change', (e) => {
    const jmaxGroup = document.getElementById('group-jmax');
    if (e.target.value === 'trap') {
        jmaxGroup.style.display = 'none';
    } else {
        jmaxGroup.style.display = '';
    }
});

document.querySelectorAll('.tuning-scroll-area input').forEach(el => {
    el.addEventListener('mousedown', () => tuningSynced = true);
});

// --- Simulation ---
let simActive = false;
let simInterval;
document.getElementById('sim-btn').addEventListener('click', () => {
    simActive = !simActive;
    const btn = document.getElementById('sim-btn');
    btn.innerText = simActive ? "Stop Sim" : "Simulate Data";
    btn.className = simActive ? "primary-btn" : "secondary-btn";

    if (simActive) {
        let t = 0;
        simInterval = setInterval(() => {
            t += 0.05;
            state.currentPos = Math.sin(t) * 180;
            state.targetPos = Math.cos(t * 0.5) * 180;
            state.vel = Math.sin(t * 1.2) * 60;
            state.acc = Math.cos(t * 2) * 150;
            state.pwm = Math.abs(Math.sin(t)) * 100;
            state.mode = "SIMULATION";

            updateUI();
            visualizer.update(state.currentPos, state.firmwareTarget);
            chartPos.addData(state.currentPos);
            chartVel.addData(state.vel);
            chartAcc.addData(state.acc);
        }, 50);
    } else {
        clearInterval(simInterval);
    }
});

// --- Tuning Mode State Machine ---
// (tuningMode is hoisted to the top of the file — see initial declaration.)
let tuningState = 'IDLE'; // IDLE | ARMED | CAPTURING | SETTLING | DONE
let tuningRunCount = 0;
let tuningStartTime = 0;
let tuningStartPos = 0;
let tuningDir = 1;
let tuningMoveTarget = 0;
let tuningSettleStart = 0;
let tuningMetricsHistory = [];

/* Mutable thresholds (configurable via gear icon on Step Response card).
 * Persisted in localStorage so they survive page reloads. */
let TUNE_VEL_THRESHOLD = 3.0;   // RPM — motor considered moving
let TUNE_POS_SETTLE_DEG = 2.0;   // degrees — within target = settling
let TUNE_VEL_SETTLE_RPM = 3.0;   // RPM — velocity settled
let TUNE_SETTLE_MS = 3000;  // ms to confirm settled

(function loadMetricsCfg() {
    try {
        const c = JSON.parse(localStorage.getItem('metricsCfg') || '{}');
        if (c.velThresh !== undefined) TUNE_VEL_THRESHOLD = c.velThresh;
        if (c.posSettle !== undefined) TUNE_POS_SETTLE_DEG = c.posSettle;
        if (c.velSettle !== undefined) TUNE_VEL_SETTLE_RPM = c.velSettle;
        if (c.settleMs !== undefined) TUNE_SETTLE_MS = c.settleMs;
    } catch (e) { }
})();

// Vel peak tracking
let tuningVelPeak = 0;
let tuningVelSetPeak = 0;

function tuningArmRun(targetOverride) {
    if (!tuningMode) return;
    tuningState = 'ARMED';
    tuningMoveTarget = targetOverride !== undefined ? targetOverride : state.firmwareTarget;
    tuningStartPos = state.currentPos;
    tuningVelPeak = 0;
    tuningVelSetPeak = 0;
    chartPos.startRun(tuningMoveTarget);
    chartVel.startRun(tuningMoveTarget);
    chartAcc.startRun(tuningMoveTarget);
    setMetricsStatus('ARMED — Waiting for motor to move...', '');
}

function tuningTick() {
    // Sine-driven capture (loop OFF): just push velocity samples for as long
    // as the sine is running. The settle state machine doesn't apply here.
    if (sineCapturing) {
        const amp = parseFloat(document.getElementById('input-sine-amp').value) || 0;
        const freq = parseFloat(document.getElementById('input-sine-freq').value) || 0;
        const t = (Date.now() - sineStartMs) / 1000;
        const ref = amp * Math.sin(2 * Math.PI * freq * t);
        chartVel.addRunPoint(state.vel, ref);
        return;
    }
    if (!tuningMode || tuningState === 'IDLE' || tuningState === 'DONE') return;

    const vel = Math.abs(state.vel);
    const posErr = Math.abs(state.currentPos - tuningMoveTarget);
    const now = Date.now();

    // Feed data to tuning charts
    if (tuningState === 'CAPTURING' || tuningState === 'SETTLING') {
        const relPos = (state.currentPos - tuningStartPos) * tuningDir;
        const relVel = state.vel * tuningDir;
        const relAcc = state.acc * tuningDir;
        const relVSet = state.velSetpoint * tuningDir;
        const relASet = state.accSetpoint * tuningDir;
        chartPos.addRunPoint(relPos, (tuningMoveTarget - tuningStartPos) * tuningDir);
        chartVel.addRunPoint(relVel, relVSet);
        chartAcc.addRunPoint(relAcc, relASet);
        tuningVelPeak = Math.max(tuningVelPeak, Math.abs(state.vel));
        tuningVelSetPeak = Math.max(tuningVelSetPeak, Math.abs(state.velSetpoint));
    }

    switch (tuningState) {
        case 'ARMED':
            if (vel > TUNE_VEL_THRESHOLD || Math.abs(state.currentPos - tuningStartPos) > 1.0) {
                tuningState = 'CAPTURING';
                tuningStartTime = now;
                tuningStartPos = state.currentPos;
                tuningDir = (tuningMoveTarget - tuningStartPos) >= 0 ? 1 : -1;
                // Reset chart runs so t=0 begins at motor start (absolute/relative target)
                const relTarget = (tuningMoveTarget - tuningStartPos) * tuningDir;
                chartPos.startRun(relTarget);
                chartVel.startRun(0);
                chartAcc.startRun(0);
                setMetricsStatus('CAPTURING...', 'capturing');
            }
            break;

        case 'CAPTURING':
            if (posErr < TUNE_POS_SETTLE_DEG && vel < TUNE_VEL_SETTLE_RPM) {
                tuningState = 'SETTLING';
                tuningSettleStart = now;
                setMetricsStatus('SETTLING...', 'settling');
            } else if (now - tuningStartTime > 15000) {
                // Auto-timeout after 15 seconds if it never settles
                tuningFinalize(now);
            }
            break;

        case 'SETTLING':
            if (posErr > TUNE_POS_SETTLE_DEG || vel > TUNE_VEL_SETTLE_RPM) {
                tuningState = 'CAPTURING'; // bounced out
                setMetricsStatus('CAPTURING...', 'capturing');
            } else if (now - tuningSettleStart >= TUNE_SETTLE_MS) {
                tuningFinalize(now);
            }
            break;
    }
}

function tuningFinalize(now) {
    tuningState = 'DONE';

    // Compute pos metrics from captured data
    const posRun = chartPos.currentRun || (chartPos.tuningRuns.length ? null : null);
    const velRunData = chartVel.currentRun;

    // Settle time = elapsed from motor start to entering settle zone
    let settleTimeSec = (tuningSettleStart - tuningStartTime) / 1000;
    if (tuningSettleStart === 0 || settleTimeSec < 0) settleTimeSec = 15.0; // Timeout fallback

    // Pos overshoot: chart vals are already in relative space (target = relTarget)
    let posOver = 0;
    const relTarget = (tuningMoveTarget - tuningStartPos) * tuningDir;
    if (chartPos.currentRun && Math.abs(relTarget) > 0.5) {
        const peak = chartPos.currentRun.vals.reduce((m, v) => Math.max(m, v), 0);
        posOver = Math.max(0, ((peak - relTarget) / relTarget) * 100);
    }

    // Vel overshoot: how much actual vel exceeded setpoint vel
    let velOver = 0;
    if (tuningVelSetPeak > 1) velOver = Math.max(0, ((tuningVelPeak - tuningVelSetPeak) / tuningVelSetPeak) * 100);

    // Vel settling: when vel dropped back within threshold
    let velSettleSec = settleTimeSec;
    if (chartVel.currentRun) {
        const vTimes = chartVel.currentRun.times;
        const vVals = chartVel.currentRun.vals;
        let lastOutIdx = -1;
        vVals.forEach((v, i) => { if (Math.abs(v) > TUNE_VEL_SETTLE_RPM) lastOutIdx = i; });
        if (lastOutIdx >= 0) velSettleSec = vTimes[lastOutIdx];
    }

    chartPos.finalizeRun(settleTimeSec, posOver);
    chartVel.finalizeRun(velSettleSec, velOver);
    chartAcc.finalizeRun(settleTimeSec, 0);

    tuningRunCount++;
    const entry = { run: tuningRunCount, posSettle: settleTimeSec, posOver, velSettle: velSettleSec, velOver };
    tuningMetricsHistory.push(entry);
    if (tuningMetricsHistory.length > 10) tuningMetricsHistory.shift();

    // Update metrics panel
    document.getElementById('m-pos-settle').innerText = settleTimeSec.toFixed(2) + 's';
    document.getElementById('m-pos-over').innerText = posOver.toFixed(1) + '%';
    document.getElementById('m-vel-settle').innerText = velSettleSec.toFixed(2) + 's';
    document.getElementById('m-vel-over').innerText = velOver.toFixed(1) + '%';
    renderMetricsHistory();

    setMetricsStatus(`Run #${tuningRunCount} done — Pos settle: ${settleTimeSec.toFixed(2)}s  OS: ${posOver.toFixed(1)}% — Trigger next move to capture again`, 'done');
}

function setMetricsStatus(msg, cls) {
    const el = document.getElementById('metrics-status');
    el.innerText = msg;
    el.className = 'metrics-status-bar ' + cls;
}

function renderMetricsHistory() {
    const tbody = document.getElementById('metrics-history-body');
    tbody.innerHTML = '';
    [...tuningMetricsHistory].reverse().forEach(e => {
        const tr = document.createElement('tr');
        tr.innerHTML = `<td>${e.run}</td><td>${e.posSettle.toFixed(2)}</td><td>${e.posOver.toFixed(1)}%</td><td>${e.velSettle.toFixed(2)}</td><td>${e.velOver.toFixed(1)}%</td>`;
        tbody.appendChild(tr);
    });
}

// Show/hide individual telemetry charts
const CHART_TOGGLES = [
    { chk: 'chk-show-pos', wrap: 'wrap-chart-pos', chart: () => chartPos },
    { chk: 'chk-show-vel', wrap: 'wrap-chart-vel', chart: () => chartVel },
    { chk: 'chk-show-acc', wrap: 'wrap-chart-acc', chart: () => chartAcc },
];
function applyChartVisibility() {
    CHART_TOGGLES.forEach(t => {
        const wrap = document.getElementById(t.wrap);
        const checked = document.getElementById(t.chk).checked;
        if (wrap) wrap.style.display = checked ? '' : 'none';
    });
    // Force the surviving canvases to re-measure their container
    requestAnimationFrame(() => {
        CHART_TOGGLES.forEach(t => {
            if (document.getElementById(t.chk).checked) t.chart().resize();
        });
    });
}
CHART_TOGGLES.forEach(t => {
    document.getElementById(t.chk).addEventListener('change', applyChartVisibility);
});
applyChartVisibility();

// View mode: 'live' | 'tuning' | 'test'
let viewMode = 'live';

function setTuningMode(on) {
    setViewMode(on ? 'tuning' : 'live');
}

function setViewMode(mode) {
    if (viewMode === mode) return;
    viewMode = mode;
    tuningMode = (mode === 'tuning');

    // Update segmented switch
    document.querySelectorAll('#view-mode-switch .vm-btn').forEach(b => {
        b.classList.toggle('active', b.dataset.mode === mode);
    });

    // Charts switch live/tuning behavior (Test reuses live scaling)
    chartPos.setMode(tuningMode);
    chartVel.setMode(tuningMode);
    chartAcc.setMode(tuningMode);
    tuningState = 'IDLE';

    // Telemetry card: show test panel instead of charts when in Test mode
    const telCard = document.querySelector('.telemetry-card');
    if (telCard) telCard.classList.toggle('test-mode', mode === 'test');
    const testPanel = document.getElementById('test-panel');
    if (testPanel) testPanel.classList.toggle('hidden', mode !== 'test');

    // Path card hides in tuning mode (preserve original behavior)
    document.querySelector('.path-card').style.display = (mode === 'tuning') ? 'none' : '';

    const _metrics = document.querySelector('.metrics-card');
    if (_metrics) _metrics.style.display = (tuningMode && posLoopEnabled) ? '' : 'none';
    const _kf = document.querySelector('.kalman-card');
    if (_kf) _kf.style.display = !posLoopEnabled ? '' : 'none';

    if (mode === 'tuning') {
        setMetricsStatus('IDLE — Send Move / Go Home / Ghost start to begin capture', '');
    } else if (mode === 'live') {
        setMetricsStatus('IDLE — Switch to Tuning Mode and move motor', '');
    }
}

// Segmented switch — wires Live / Tuning / Test buttons
document.querySelectorAll('#view-mode-switch .vm-btn').forEach(btn => {
    btn.addEventListener('click', () => setViewMode(btn.dataset.mode));
});

// Default: hide metrics card on load (Live mode default)
document.addEventListener('DOMContentLoaded', () => {
    const m = document.querySelector('.metrics-card');
    if (m) m.style.display = 'none';
});

/* ============================================================================
 * Kalman Filter dashboard
 * ========================================================================== */

/* Relocate the Kalman card from the right column to the left column so it
 * lives next to (and replaces) the Step Response Metrics panel. */
(function relocateKalmanCard() {
    const kf = document.querySelector('.kalman-card');
    const metrics = document.querySelector('.metrics-card');
    if (kf && metrics && metrics.parentElement) {
        metrics.parentElement.insertBefore(kf, metrics.nextSibling);
        kf.style.display = 'none';  // visibility is driven by Loop / Tuning state
    }
})();
const KF_RMSE_WINDOW = 100;         // last N samples for RMSE
const kfRmseTheta = [];
const kfRmseOmega = [];

function updateKalmanCard() {
    const set = (id, txt) => { const el = document.getElementById(id); if (el) el.innerText = txt; };
    set('kf-theta', state.kfTheta.toFixed(2));
    set('kf-omega', state.kfOmega.toFixed(2));
    set('kf-tau', state.kfTau.toFixed(3));
    set('kf-ia', state.kfIa.toFixed(3));
    set('kf-innov', state.kfInnov.toExponential(2));
    set('kf-p00', state.kfP00.toExponential(2));
    set('kf-p11', state.kfP11.toExponential(2));
    set('kf-p22', state.kfP22.toExponential(2));

    // RMSE: compare KF estimate to the raw signal we'd otherwise use.
    // For pos: kfTheta vs raw encoder (state.currentPos).
    // For vel: kfOmega vs lowpass-derived RPM (state.vel).
    kfRmseTheta.push(state.kfTheta - state.currentPos);
    kfRmseOmega.push(state.kfOmega - state.vel);
    if (kfRmseTheta.length > KF_RMSE_WINDOW) kfRmseTheta.shift();
    if (kfRmseOmega.length > KF_RMSE_WINDOW) kfRmseOmega.shift();
    const rms = arr => {
        if (!arr.length) return 0;
        let s = 0; for (const v of arr) s += v * v;
        return Math.sqrt(s / arr.length);
    };
    set('kf-rmse-theta', rms(kfRmseTheta).toFixed(3) + ' °');
    set('kf-rmse-omega', rms(kfRmseOmega).toFixed(2) + ' RPM');

    // Toggle button reflects firmware state (so the dashboard syncs after reload)
    const btn = document.getElementById('btn-kf-toggle');
    if (btn) {
        btn.textContent = state.kfEnabled ? 'KF: ON' : 'KF: OFF';
        btn.classList.toggle('active', state.kfEnabled);
        btn.classList.toggle('warning', !state.kfEnabled);
    }
}

document.getElementById('btn-kf-toggle').addEventListener('click', () => {
    const next = !state.kfEnabled;
    sendCommand(`SET:KF_EN=${next ? 1 : 0}`);
    log(`Kalman filter ${next ? 'ENABLED' : 'DISABLED'}`);
});

document.getElementById('btn-kf-reset').addEventListener('click', () => {
    sendCommand('SET:KF_RESET=1');
    kfRmseTheta.length = 0;
    kfRmseOmega.length = 0;
    log('Kalman filter reset to current encoder reading');
});

document.getElementById('btn-kf-sanity').addEventListener('click', () => {
    state.kfSanityShow = !state.kfSanityShow;
    const b = document.getElementById('btn-kf-sanity');
    b.textContent = state.kfSanityShow ? 'Model: ON' : 'Model: OFF';
    b.classList.toggle('active', state.kfSanityShow);
    b.classList.toggle('warning', !state.kfSanityShow);
    if (!state.kfSanityShow) {
        chartPos.clearSanity();
        chartVel.clearSanity();
    }
});

document.getElementById('btn-kf-apply-noise').addEventListener('click', () => {
    const send = (key, id) => {
        const v = parseFloat(document.getElementById(id).value);
        if (!isNaN(v)) sendCommand(`SET:${key}=${v}`);
    };
    send('KF_Q_THETA', 'input-kf-q-theta');
    send('KF_Q_OMEGA', 'input-kf-q-omega');
    send('KF_Q_TAU', 'input-kf-q-tau');
    send('KF_Q_I', 'input-kf-q-i');
    send('KF_R', 'input-kf-r');
    log('Kalman noise parameters applied');
});

// --- Gripper Config ---
const gripperConfig = {
    enabled: false,
    mode: 'sim',
    delays: { open: 600, close: 600, up: 600, down: 600 },
    realTimeout: 5000,
};

document.getElementById('chk-use-gripper').addEventListener('change', e => {
    gripperConfig.enabled = e.target.checked;
});

document.getElementById('btn-gripper-settings').addEventListener('click', () => {
    document.getElementById('gripper-modal').classList.remove('hidden');
});
document.getElementById('btn-close-gripper-modal').addEventListener('click', () => {
    document.getElementById('gripper-modal').classList.add('hidden');
    gripperConfig.delays.open = parseInt(document.getElementById('delay-open').value) || 600;
    gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
    gripperConfig.delays.up = parseInt(document.getElementById('delay-up').value) || 600;
    gripperConfig.delays.down = parseInt(document.getElementById('delay-down').value) || 600;
});
document.getElementById('gripper-modal').addEventListener('click', e => {
    if (e.target.id === 'gripper-modal') {
        e.target.classList.add('hidden');
        gripperConfig.delays.open = parseInt(document.getElementById('delay-open').value) || 600;
        gripperConfig.delays.close = parseInt(document.getElementById('delay-close').value) || 600;
        gripperConfig.delays.up = parseInt(document.getElementById('delay-up').value) || 600;
        gripperConfig.delays.down = parseInt(document.getElementById('delay-down').value) || 600;
    }
});
document.querySelectorAll('input[name="gripper-mode"]').forEach(radio => {
    radio.addEventListener('change', e => {
        gripperConfig.mode = e.target.value;
        document.getElementById('sim-delays-section').style.display = gripperConfig.mode === 'sim' ? '' : 'none';
        document.getElementById('real-sensor-info').style.display = gripperConfig.mode === 'real' ? '' : 'none';
    });
});

// --- Gripper Sequence Helpers ---
function msDelay(ms) {
    // Guard: NaN / undefined / negative → setTimeout fires immediately, which
    // causes the path sequencer to skip its inter-waypoint wait.
    const safe = (Number.isFinite(ms) && ms > 0) ? ms : 0;
    return new Promise(r => setTimeout(r, safe));
}

function waitForGripperState(predicate, timeout) {
    return new Promise(resolve => {
        if (predicate()) { resolve(); return; }
        const t0 = Date.now();
        const iv = setInterval(() => {
            if (predicate() || Date.now() - t0 >= timeout) { clearInterval(iv); resolve(); }
        }, 50);
    });
}

async function gripperStep(cmd, confirmFn, simMs) {
    sendCommand(cmd);
    if (gripperConfig.mode === 'real') {
        await waitForGripperState(confirmFn, gripperConfig.realTimeout);
    } else {
        await msDelay(simMs);
    }
    gripperVisualizer.update(!!state.gripper_ud, !state.gripper_co);
}

async function runGripperPick() {
    await gripperStep('CMD:GRIP_OPEN', () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_DN', () => state.gripper_ud, gripperConfig.delays.down);
    await gripperStep('CMD:GRIP_CLOSE', () => state.gripper_co, gripperConfig.delays.close);
    await gripperStep('CMD:GRIP_UP', () => !state.gripper_ud, gripperConfig.delays.up);
}

async function runGripperPlace() {
    await gripperStep('CMD:GRIP_DN', () => state.gripper_ud, gripperConfig.delays.down);
    await gripperStep('CMD:GRIP_OPEN', () => !state.gripper_co, gripperConfig.delays.open);
    await gripperStep('CMD:GRIP_UP', () => !state.gripper_ud, gripperConfig.delays.up);
    await gripperStep('CMD:GRIP_CLOSE', () => state.gripper_co, gripperConfig.delays.close);
}

// --- Path Sequencer (Live mode) ---
const waypointList = document.getElementById('waypoint-list');
const btnRunSeq = document.getElementById('btn-run-seq');
const btnLoopSeq = document.getElementById('btn-loop-seq');
const inputNewWaypoint = document.getElementById('input-new-waypoint');

document.getElementById('btn-add-waypoint').addEventListener('click', () => {
    const val = parseFloat(inputNewWaypoint.value);
    if (!isNaN(val)) {
        state.waypoints.push(val);
        inputNewWaypoint.value = "";
        renderWaypoints();
    }
});

btnRunSeq.addEventListener('click', () => {
    state.seqActive = !state.seqActive;
    btnRunSeq.innerText = state.seqActive ? 'Stop' : 'Run';
    btnRunSeq.className = 'toggle-btn ' + (state.seqActive ? 'active' : '');
    if (state.seqActive) {
        state.currentWaypointIdx = 0;
        state.gripperHasRod = false;  // start each Run with empty gripper
        executeNextWaypoint();
    }
});

btnLoopSeq.addEventListener('click', () => {
    state.seqLoop = !state.seqLoop;
    btnLoopSeq.className = 'toggle-btn ' + (state.seqLoop ? 'active' : '');
});

function renderWaypoints() {
    waypointList.innerHTML = '';
    state.waypoints.forEach((wp, i) => {
        const li = document.createElement('li');
        if (i === state.currentWaypointIdx) li.className = 'active';
        li.innerHTML = `<span>${wp}°</span><span class="remove-waypoint" onclick="removeWaypoint(${i})">×</span>`;
        waypointList.appendChild(li);
    });
}

window.removeWaypoint = (i) => { state.waypoints.splice(i, 1); renderWaypoints(); };

async function executeNextWaypoint() {
    if (!state.seqActive || state.waypoints.length === 0) return;
    const target = state.waypoints[state.currentWaypointIdx];
    sendCommand(`SET:TARGET=${target}`);
    renderWaypoints();

    const rawDelay = parseFloat(document.getElementById('input-seq-delay').value);
    const movDelay = (Number.isFinite(rawDelay) && rawDelay >= 0) ? rawDelay * 1000 : 2000;
    await msDelay(movDelay);
    if (!state.seqActive) return;

    if (gripperConfig.enabled) {
        // Single-rod shuttle. Action at each waypoint depends on whether the
        // gripper is currently holding the rod, not on waypoint index — so it
        // works across loops without dropping the rod.
        //
        //   empty gripper  → PICK (grab here)
        //   holding rod, last waypoint, no loop → PLACE (final drop)
        //   holding rod, otherwise              → PLACE then PICK
        const idx = state.currentWaypointIdx;
        const isLast = (idx === state.waypoints.length - 1);

        if (!state.gripperHasRod) {
            await runGripperPick();
            state.gripperHasRod = true;
        } else if (isLast && !state.seqLoop) {
            await runGripperPlace();
            state.gripperHasRod = false;
        } else {
            await runGripperPlace();
            state.gripperHasRod = false;
            if (!state.seqActive) return;
            await runGripperPick();
            state.gripperHasRod = true;
        }
    }
    if (!state.seqActive) return;

    state.currentWaypointIdx++;
    if (state.currentWaypointIdx >= state.waypoints.length) {
        if (state.seqLoop) {
            state.currentWaypointIdx = 0;
        } else {
            state.seqActive = false;
            state.currentWaypointIdx = -1;
            btnRunSeq.innerText = 'Run';
            btnRunSeq.className = 'toggle-btn';
            renderWaypoints();
            return;
        }
    }
    executeNextWaypoint();
}

// Auto-arm capture on ghost-mode rising edge
let _lastGhost = false;
function checkGhostStart() {
    if (tuningMode && !_lastGhost && state.ghost && tuningState === 'IDLE') {
        tuningArmRun();
        setMetricsStatus('ARMED (Ghost replay) — Waiting for motor motion...', '');
    }
    _lastGhost = state.ghost;
}

// --- Default tuning values (mirrors params.h) ---
const DEFAULTS = {
    'input-speed-kp': 1.0,
    'input-speed-ki': 2.0,
    'input-speed-kd': 0.0,
    'input-k-vff': 0.0,
    'input-k-aff': 0.0,
    'input-pos-kp': 0.4,
    'input-pos-ki': 0.05,
    'input-pos-kd': 0.1,
    'input-min-pwm': 0.0,
    'input-home-speed': 30,
    'input-jog-fine': 10,
    'input-vmax-rad': 7.304,
    'input-amax-rad': 27.49,
    'input-jmax-rad': 1400,
    'input-step-coarse': 10,
    'input-step-fine': 1.0,
};

function applyDefaults() {
    for (const [id, val] of Object.entries(DEFAULTS)) {
        const el = document.getElementById(id);
        if (el) el.value = val;
    }
}

// --- Auto-reconnect on page load ---
window.addEventListener('DOMContentLoaded', async () => {
    applyDefaults();
    gripperVisualizer.resize();
    // Auto-connect to the WebSocket bridge on page load
    log("Auto-connecting to WebSocket bridge...");
    connectSerial();
});

updateUI();
