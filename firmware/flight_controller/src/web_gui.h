#ifndef WEB_GUI_H
#define WEB_GUI_H

/* ===========================================================================
   Embedded Web GUI for Wireless PID Tuning
   ===========================================================================
   Served by the ESP32-S3 WebSocket HTTP server on port 80.
   PC browser connects to http://192.168.4.1 and tunes PID parameters
   wirelessly via WebSocket on port 81.
   =========================================================================== */

static const char WEB_GUI_HTML[] =
R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>IntelQCopter - Wireless PID Tuner</title>
<style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
        font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        background: linear-gradient(135deg, #0f172a 0%, #1e293b 100%);
        color: #e2e8f0;
        min-height: 100vh;
        padding: 20px;
    }
    .container {
        max-width: 1200px;
        margin: 0 auto;
    }
    .header {
        text-align: center;
        margin-bottom: 30px;
        padding: 20px;
        background: rgba(30, 41, 59, 0.8);
        border-radius: 12px;
        border: 1px solid #334155;
    }
    .header h1 { color: #38bdf8; font-size: 2.5rem; margin-bottom: 8px; }
    .header p { color: #94a3b8; font-size: 1rem; }
    .status-bar {
        display: flex;
        justify-content: space-between;
        align-items: center;
        margin-bottom: 20px;
        padding: 12px 20px;
        background: rgba(30, 41, 59, 0.8);
        border-radius: 12px;
        border: 1px solid #334155;
    }
    .status-indicator {
        display: flex;
        align-items: center;
        gap: 10px;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; }
    .dot.connected { background: #22c55e; box-shadow: 0 0 10px #22c55e; }
    .dot.disconnected { background: #ef4444; box-shadow: 0 0 10px #ef4444; }
    .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
        gap: 20px;
        margin-bottom: 30px;
    }
    .card {
        background: rgba(30, 41, 59, 0.6);
        border: 1px solid #334155;
        border-radius: 12px;
        padding: 20px;
        backdrop-filter: blur(10px);
    }
    .card h2 {
        color: #38bdf8;
        margin-bottom: 16px;
        font-size: 1.25rem;
        border-bottom: 1px solid #334155;
        padding-bottom: 8px;
    }
    .telemetry-row {
        display: flex;
        justify-content: space-between;
        margin: 8px 0;
        font-size: 0.95rem;
    }
    .telemetry-label { color: #94a3b8; }
    .telemetry-value { color: #e2e8f0; font-weight: 600; font-family: 'Courier New', monospace; }
    .telemetry-value.ok { color: #22c55e; }
    .telemetry-value.warn { color: #f59e0b; }
    .telemetry-value.error { color: #ef4444; }
    .pid-group {
        margin-bottom: 16px;
    }
    .pid-group label {
        display: block;
        color: #94a3b8;
        margin-bottom: 6px;
        font-size: 0.85rem;
        text-transform: uppercase;
        letter-spacing: 0.5px;
    }
    .pid-row {
        display: flex;
        gap: 10px;
        margin-bottom: 10px;
    }
    .pid-input {
        flex: 1;
        padding: 8px 12px;
        background: rgba(15, 23, 42, 0.8);
        border: 1px solid #475569;
        border-radius: 6px;
        color: #e2e8f0;
        font-size: 0.95rem;
        font-family: 'Courier New', monospace;
    }
    .pid-input:focus {
        outline: none;
        border-color: #38bdf8;
    }
    .pid-input.invalid { border-color: #ef4444; }
    .pid-axis-name {
        font-weight: 600;
        color: #38bdf8;
        font-size: 0.9rem;
        margin-bottom: 8px;
        display: block;
    }
    .btn-group {
        display: flex;
        gap: 10px;
        margin-top: 16px;
    }
    .btn {
        padding: 10px 20px;
        border: none;
        border-radius: 6px;
        color: #fff;
        font-size: 0.95rem;
        font-weight: 600;
        cursor: pointer;
        transition: all 0.2s;
        flex: 1;
    }
    .btn:hover { opacity: 0.9; transform: translateY(-1px); }
    .btn:active { transform: translateY(0); }
    .btn-primary { background: linear-gradient(135deg, #38bdf8, #0ea5e9); }
    .btn-danger { background: linear-gradient(135deg, #ef4444, #dc2626); }
    .btn-success { background: linear-gradient(135deg, #22c55e, #16a34a); }
    .btn:disabled { opacity: 0.5; cursor: not-allowed; }
    .armed-indicator {
        display: inline-block;
        padding: 6px 16px;
        border-radius: 20px;
        font-size: 0.85rem;
        font-weight: 700;
    }
    .armed { background: rgba(34, 197, 94, 0.2); color: #22c55e; border: 1px solid #22c55e; }
    .disarmed { background: rgba(239, 68, 68, 0.2); color: #ef4444; border: 1px solid #ef4444; }
    .footer {
        text-align: center;
        margin-top: 30px;
        padding: 20px;
        color: #64748b;
        font-size: 0.85rem;
    }
    .alert {
        padding: 12px 16px;
        border-radius: 6px;
        margin-bottom: 20px;
        font-size: 0.9rem;
    }
    .alert.info { background: rgba(56, 189, 248, 0.1); border: 1px solid #38bdf8; color: #38bdf8; }
    .alert.warn { background: rgba(245, 158, 11, 0.1); border: 1px solid #f59e0b; color: #f59e0b; }
    .alert.error { background: rgba(239, 68, 68, 0.1); border: 1px solid #ef4444; color: #ef4444; }
    .hidden { display: none; }
</style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>IntelQCopter</h1>
        <p>Wireless PID Tuner &amp; Telemetry</p>
    </div>

    <div id="alert-area"></div>

    <div class="status-bar">
        <div class="status-indicator">
            <div class="dot disconnected" id="ws-dot"></div>
            <span id="ws-status">Disconnected</span>
        </div>
        <div>
            <strong>IP:</strong> <span id="ip-addr">192.168.4.1</span> |
            <strong>Port:</strong> 81 |
            <strong>Loop:</strong> <span id="loop-rate">--</span> Hz
        </div>
    </div>

    <div class="grid">
        <!-- Attitude -->
        <div class="card">
            <h2>Attitude</h2>
            <div class="telemetry-row">
                <span class="telemetry-label">Roll</span>
                <span class="telemetry-value" id="tel-roll">0.00°</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Pitch</span>
                <span class="telemetry-value" id="tel-pitch">0.00°</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Yaw</span>
                <span class="telemetry-value" id="tel-yaw">0.00°</span>
            </div>
        </div>

        <!-- Rates -->
        <div class="card">
            <h2>Angular Rates</h2>
            <div class="telemetry-row">
                <span class="telemetry-label">Roll Rate</span>
                <span class="telemetry-value" id="tel-roll-rate">0.00°/s</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Pitch Rate</span>
                <span class="telemetry-value" id="tel-pitch-rate">0.00°/s</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Yaw Rate</span>
                <span class="telemetry-value" id="tel-yaw-rate">0.00°/s</span>
            </div>
        </div>

        <!-- Motors -->
        <div class="card">
            <h2>Motor Outputs</h2>
            <div class="telemetry-row">
                <span class="telemetry-label">FL</span>
                <span class="telemetry-value" id="tel-motor0">0.00</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">FR</span>
                <span class="telemetry-value" id="tel-motor1">0.00</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">RR</span>
                <span class="telemetry-value" id="tel-motor2">0.00</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">RL</span>
                <span class="telemetry-value" id="tel-motor3">0.00</span>
            </div>
        </div>

        <!-- Flight State -->
        <div class="card">
            <h2>Flight State</h2>
            <div class="telemetry-row">
                <span class="telemetry-label">Armed</span>
                <span class="armed-indicator disarmed" id="state-armed">DISARMED</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Altitude</span>
                <span class="telemetry-value" id="tel-altitude">0.00 m</span>
            </div>
            <div class="telemetry-row">
                <span class="telemetry-label">Vert Speed</span>
                <span class="telemetry-value" id="tel-vert-speed">0.00 m/s</span>
            </div>
        </div>
    </div>

    <!-- PID Tuning Cards -->
    <div class="grid">
        <div class="card">
            <h2>Roll Angle PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Roll Angle</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-roll-angle-kp" step="0.01" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-roll-angle-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-roll-angle-kd" step="0.001" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('roll_angle')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Pitch Angle PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Pitch Angle</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-pitch-angle-kp" step="0.01" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-pitch-angle-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-pitch-angle-kd" step="0.001" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('pitch_angle')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Roll Rate PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Roll Rate</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-roll-rate-kp" step="0.001" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-roll-rate-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-roll-rate-kd" step="0.0001" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('roll_rate')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Pitch Rate PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Pitch Rate</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-pitch-rate-kp" step="0.001" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-pitch-rate-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-pitch-rate-kd" step="0.0001" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('pitch_rate')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Yaw Rate PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Yaw Rate</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-yaw-rate-kp" step="0.001" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-yaw-rate-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-yaw-rate-kd" step="0.0001" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('yaw_rate')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Altitude PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Altitude</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-altitude-kp" step="0.01" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-altitude-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-altitude-kd" step="0.01" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('altitude')">Apply</button>
            </div>
        </div>

        <div class="card">
            <h2>Vertical Speed PID</h2>
            <div class="pid-group">
                <span class="pid-axis-name">Vertical Speed</span>
                <div class="pid-row">
                    <input type="number" class="pid-input" id="pid-vert-speed-kp" step="0.01" placeholder="Kp">
                    <input type="number" class="pid-input" id="pid-vert-speed-ki" step="0.001" placeholder="Ki">
                    <input type="number" class="pid-input" id="pid-vert-speed-kd" step="0.01" placeholder="Kd">
                </div>
                <button class="btn btn-primary" onclick="sendPid('vert_speed')">Apply</button>
            </div>
        </div>
    </div>

    <div class="btn-group" style="justify-content: center; max-width: 600px; margin: 0 auto;">
        <button class="btn btn-success" onclick="requestAllPids()">Request All PIDs</button>
        <button class="btn btn-danger" onclick="emergencyDisarm()">EMERGENCY DISARM</button>
    </div>

    <div class="footer">
        IntelQCopter Wireless PID Tuner &bull; ESP32-S3 WiFi AP &bull; WebSocket: ws://192.168.4.1:81
    </div>
</div>

<script>
let ws = null;
let reconnectTimer = null;
let lastTelemetryTime = 0;

function connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const host = window.location.hostname;
    const wsUrl = protocol + '//' + host + ':81';
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        document.getElementById('ws-dot').className = 'dot connected';
        document.getElementById('ws-status').textContent = 'Connected';
        showAlert('Connected to flight controller', 'info');
        requestAllPids();
    };
    
    ws.onclose = () => {
        document.getElementById('ws-dot').className = 'dot disconnected';
        document.getElementById('ws-status').textContent = 'Disconnected';
        showAlert('Disconnected from flight controller', 'warn');
        ws = null;
        reconnectTimer = setTimeout(connect, 2000);
    };
    
    ws.onerror = (err) => {
        console.error('WebSocket error:', err);
        showAlert('WebSocket error', 'error');
    };
    
    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            handleMessage(msg);
        } catch (e) {
            console.error('Failed to parse message:', e);
        }
    };
}

function handleMessage(msg) {
    switch (msg.type) {
        case 'telemetry':
            updateTelemetry(msg);
            break;
        case 'pid_values':
            updatePidInputs(msg);
            break;
        case 'pid_ack':
            showAlert('PID updated: ' + msg.axis, 'info');
            break;
        case 'error':
            showAlert(msg.message, 'error');
            break;
        default:
            console.log('Unknown message type:', msg.type);
    }
}

function updateTelemetry(t) {
    document.getElementById('tel-roll').textContent = (t.roll || 0).toFixed(2) + '°';
    document.getElementById('tel-pitch').textContent = (t.pitch || 0).toFixed(2) + '°';
    document.getElementById('tel-yaw').textContent = (t.yaw || 0).toFixed(2) + '°';
    document.getElementById('tel-roll-rate').textContent = (t.roll_rate || 0).toFixed(2) + '°/s';
    document.getElementById('tel-pitch-rate').textContent = (t.pitch_rate || 0).toFixed(2) + '°/s';
    document.getElementById('tel-yaw-rate').textContent = (t.yaw_rate || 0).toFixed(2) + '°/s';
    
    const armedEl = document.getElementById('state-armed');
    if (t.armed) {
        armedEl.className = 'armed-indicator armed';
        armedEl.textContent = 'ARMED';
    } else {
        armedEl.className = 'armed-indicator disarmed';
        armedEl.textContent = 'DISARMED';
    }
    
    document.getElementById('tel-altitude').textContent = (t.altitude || 0).toFixed(2) + ' m';
    document.getElementById('tel-vert-speed').textContent = (t.vert_speed || 0).toFixed(2) + ' m/s';
    
    if (t.motor && t.motor.length === 4) {
        document.getElementById('tel-motor0').textContent = (t.motor[0] || 0).toFixed(3);
        document.getElementById('tel-motor1').textContent = (t.motor[1] || 0).toFixed(3);
        document.getElementById('tel-motor2').textContent = (t.motor[2] || 0).toFixed(3);
        document.getElementById('tel-motor3').textContent = (t.motor[3] || 0).toFixed(3);
    }
    
    if (t.loop_rate) {
        document.getElementById('loop-rate').textContent = t.loop_rate;
    }
    
    lastTelemetryTime = Date.now();
}

function updatePidInputs(pids) {
    const map = {
        'roll_angle':   ['roll-angle',   ['kp','ki','kd']],
        'pitch_angle':  ['pitch-angle',  ['kp','ki','kd']],
        'roll_rate':    ['roll-rate',    ['kp','ki','kd']],
        'pitch_rate':   ['pitch-rate',   ['kp','ki','kd']],
        'yaw_rate':     ['yaw-rate',     ['kp','ki','kd']],
        'altitude':     ['altitude',     ['kp','ki','kd']],
        'vert_speed':   ['vert-speed',   ['kp','ki','kd']],
    };
    
    for (const [axis, info] of Object.entries(map)) {
        if (pids[axis]) {
            const [idPrefix, gains] = info;
            gains.forEach(g => {
                const el = document.getElementById('pid-' + idPrefix + '-' + g);
                if (el) el.value = pids[axis][g];
            });
        }
    }
}

function sendPid(axis) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showAlert('Not connected to flight controller', 'warn');
        return;
    }
    
    const gains = {};
    let valid = true;
    
    ['kp', 'ki', 'kd'].forEach(g => {
        const el = document.getElementById('pid-' + axis + '-' + g);
        const val = parseFloat(el.value);
        if (isNaN(val)) {
            el.classList.add('invalid');
            valid = false;
        } else {
            el.classList.remove('invalid');
            gains[g] = val;
        }
    });
    
    if (!valid) {
        showAlert('Invalid PID values entered', 'error');
        return;
    }
    
    const msg = {
        type: 'pid_set',
        axis: axis,
        kp: gains.kp,
        ki: gains.ki,
        kd: gains.kd
    };
    
    ws.send(JSON.stringify(msg));
    showAlert('Sent PID update for ' + axis, 'info');
}

function requestAllPids() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showAlert('Not connected to flight controller', 'warn');
        return;
    }
    ws.send(JSON.stringify({ type: 'pid_get' }));
}

function emergencyDisarm() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        showAlert('Not connected to flight controller', 'warn');
        return;
    }
    if (confirm('EMERGENCY DISARM - All motors will stop immediately. Continue?')) {
        ws.send(JSON.stringify({ type: 'disarm' }));
        showAlert('Emergency disarm sent', 'error');
    }
}

function showAlert(msg, type) {
    const area = document.getElementById('alert-area');
    const div = document.createElement('div');
    div.className = 'alert ' + type;
    div.textContent = msg;
    area.appendChild(div);
    setTimeout(() => div.remove(), 5000);
}

// Auto-connect on page load
connect();

// Auto-reconnect if connection lost
setInterval(() => {
    if (!ws && !reconnectTimer) {
        connect();
    }
}, 3000);

</script>
)rawliteral";

#endif /* WEB_GUI_H */