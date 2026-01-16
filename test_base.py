#!/usr/bin/env python3
# Minimal Flask web UI: RS-485 byte buttons + RPIPLC analog output + Home + Plates + Valves + Pump timer + Roller
# NOTE: PAGE is a plain triple-quoted string (not an f-string) to avoid { } conflicts with JS.

from flask import Flask, request, jsonify, render_template_string
import time
import threading
from contextlib import contextmanager

SERIAL_LOCK = threading.Lock()

import commands  # your module with ser + write(...)

# If your stepper driver needs a different baud for homing. None keeps current.
HOMING_BAUD = None  # e.g., 115200 or 57600

@contextmanager
def temp_baudrate(serial_obj, new_baud):
    old = serial_obj.baudrate
    try:
        if new_baud and old != new_baud:
            try:
                serial_obj.reset_input_buffer()
                serial_obj.reset_output_buffer()
            except Exception:
                pass
            serial_obj.baudrate = new_baud
            time.sleep(0.05)
        yield
    finally:
        if new_baud and serial_obj.baudrate != old:
            try:
                serial_obj.reset_input_buffer()
                serial_obj.reset_output_buffer()
            except Exception:
                pass
            serial_obj.baudrate = old
            time.sleep(0.05)

# RPIPLC (optional; only needed for analog output)
try:
    from librpiplc import rpiplc
    HAVE_RPIPLC = True
    RPIPLC_IMPORT_ERR = ""
except Exception as e:
    rpiplc = None
    HAVE_RPIPLC = False
    RPIPLC_IMPORT_ERR = str(e)

# ---- Command bytes ----
CMD = {
    # Movement / pump / base
    "move_base_front_on":   b'\x01\x06\x00\x08\x01\x00\x09\x98',
    "move_base_front_off":  b'\x01\x06\x00\x08\x02\x00\x09\x68',
    "pump_start":           b'\x01\x06\x00\x06\x01\x00\x68\x5B',
    "pump_stop":            b'\x01\x06\x00\x06\x02\x00\x68\xAB',
    "base_back_on":         b'\x01\x06\x00\x04\x01\x00\xC9\x9B',
    "base_back_off":        b'\x01\x06\x00\x04\x02\x00\xC9\x6B',

    # Roller (relay 7)
    "roller_start":         b'\x01\x06\x00\x07\x01\x00\x39\x9B',
    "roller_stop":          b'\x01\x06\x00\x07\x02\x00\x39\x6B',

    # Valves (OPEN/CLOSE)
    "valve1_open":          b'\x01\x06\x00\x01\x01\x00\xD9\x9A',
    "valve1_close":         b'\x01\x06\x00\x01\x02\x00\xD9\x6A',

    "valve2_open":          b'\x01\x06\x00\x02\x01\x00\x29\x9A',
    "valve2_close":         b'\x01\x06\x00\x02\x02\x00\x29\x6A',

    "valve3_open":          b'\x01\x06\x00\x03\x01\x00\x78\x5A',
    "valve3_close":         b'\x01\x06\x00\x03\x02\x00\x78\xAA',

    "valve4_open":          b'\x01\x06\x00\x04\x01\x00\xC9\x9B',
    "valve4_close":         b'\x01\x06\x00\x04\x02\x00\xC9\x6B',

    "valve5_open":          b'\x01\x06\x00\x05\x01\x00\x98\x5B',
    "valve5_close":         b'\x01\x06\x00\x05\x02\x00\x98\xAB',
}

# Homing commands
command_homing1 = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x00\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\xF8\x65'
command_homing2 = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x02\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\x01\xA2'

# Plates
command_stepper_open  = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x00\x00\x00\x33\x58'
command_stepper_press = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x89\x54\x40\xDD\x82'
# 23 bytes



# Roller bump duration
ROLLER_BUMP_SECONDS = 6

app = Flask(__name__)

PAGE = """
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>ENVIROMED Quick Controls — Roller Build</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  body { font-family: system-ui, sans-serif; margin: 20px; }
  h1 { margin-bottom: 12px; }
  .grid { display: grid; grid-template-columns: 1fr auto auto auto; gap: 8px 12px; max-width: 1000px; }
  .rowlabel { padding: 10px 0; }
  button { padding: 8px 14px; border: 0; border-radius: 10px; cursor: pointer; }
  .on { background: #e6f4ea; }
  .off { background: #fdeaea; }
  .start { background: #e6f0ff; }
  .stop { background: #fff0f0; }
  .home { background: #fff7e6; }
  .press { background: #ffe6f0; }
  .open { background: #e6fff3; }
  .pill { display:inline-block; padding:6px 10px; border-radius: 999px; border:1px solid #ddd; min-width:90px; text-align:center; font-variant-numeric: tabular-nums; }
  #log { margin-top: 16px; white-space: pre-wrap; background:#f6f6f6; padding:10px; border-radius:10px; max-width: 1000px; min-height: 160px;}
  .row { display:flex; align-items:center; gap:8px; }
  input[type="number"] { width: 100px; padding:6px 8px; border-radius:8px; border:1px solid #ddd; }
  .muted { color:#666; font-size: 0.9rem; }
  .section { margin-top: 18px; font-weight: 600; }
</style>
<script>
let pumpTimerInterval = null;
let pumpStartEpoch = null;

function fmtHMS(totalSeconds){
  totalSeconds = Math.max(0, Math.floor(totalSeconds));
  const h = Math.floor(totalSeconds / 3600);
  const m = Math.floor((totalSeconds % 3600) / 60);
  const s = totalSeconds % 60;
  const two = (n)=> String(n).padStart(2,'0');
  return two(h)+":"+two(m)+":"+two(s);
}

function startPumpTimer(){
  const el = document.getElementById("pumpTimer");
  pumpStartEpoch = Date.now();
  el.textContent = "00:00:00";
  if (pumpTimerInterval) clearInterval(pumpTimerInterval);
  pumpTimerInterval = setInterval(()=>{
    const elapsed = (Date.now() - pumpStartEpoch)/1000.0;
    el.textContent = fmtHMS(elapsed);
  }, 1000);
}

function stopPumpTimer(){
  if (pumpTimerInterval){
    clearInterval(pumpTimerInterval);
    pumpTimerInterval = null;
  }
}

async function send(action, label){
  const r = await fetch("/send", {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify({action})});
  const j = await r.json();
  const ts = new Date().toLocaleTimeString();
  const line = `[${ts}] ${j.ok ? "OK" : "ERR"} ${label}  ${j.hex || ""}  ${j.error || ""}\\n`;
  const log = document.getElementById("log");
  log.textContent += line;
  log.scrollTop = log.scrollHeight;

  if (j.ok && action === "pump_start"){
    startPumpTimer();
  }
  if (j.ok && action === "pump_stop"){
    stopPumpTimer();
  }
}

async function setAnalog(){
  const val = document.getElementById("dacval").value;
  const r = await fetch("/set_analog", {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify({pin:"A0.5", value: Number(val)})});
  const j = await r.json();
  const ts = new Date().toLocaleTimeString();
  const line = `[${ts}] ${j.ok ? "OK" : "ERR"} Set A0.5 = ${val}  ${j.error || ""}\\n`;
  const log = document.getElementById("log");
  log.textContent += line;
  log.scrollTop = log.scrollHeight;
}

async function home(){
  const r = await fetch("/home", {method:"POST"});
  const j = await r.json();
  const ts = new Date().toLocaleTimeString();
  const line = `[${ts}] ${j.ok ? "OK" : "ERR"} Home  ${j.detail || ""} ${j.error || ""}\\n`;
  const log = document.getElementById("log");
  log.textContent += line;
  log.scrollTop = log.scrollHeight;
}
</script>
</head>
<body>
  <h1>ENVIROMED — Quick Controls</h1>
  <p class="muted">RS-485 bytes via <code>commands.ser.write(...)</code> & <code>commands.write(...)</code> • RPIPLC analog output via <code>librpiplc</code></p>

  <div class="section">Movement & Pump</div>
  <div class="grid">
    <div class="rowlabel">Move Base Front</div>
    <button class="on"  onclick="send('move_base_front_on','Move Base Front ON')">ON</button>
    <button class="off" onclick="send('move_base_front_off','Move Base Front OFF')">OFF</button>
    <div></div>

    <div class="rowlabel">Peristaltic Pump</div>
    <button class="start" onclick="send('pump_start','Pump START')">START</button>
    <button class="stop"  onclick="send('pump_stop','Pump STOP')">STOP</button>
    <span id="pumpTimer" class="pill">00:00:00</span>

    <div class="rowlabel">Roller (Relay 7)</div>
    <button class="start" onclick="send('roller_start','Roller START')">START</button>
    <button class="stop"  onclick="send('roller_stop','Roller STOP')">STOP</button>
    <button class="home"  onclick="send('roller_bump','Roller BUMP 6s')">BUMP 6s</button>

    <div class="rowlabel">Base Back</div>
    <button class="on"  onclick="send('base_back_on','Base Back ON')">ON</button>
    <button class="off" onclick="send('base_back_off','Base Back OFF')">OFF</button>
    <div></div>

    <div class="rowlabel">Homing</div>
    <button class="home" onclick="home()">HOME (send 1→2)</button>
    <div></div>
    <div></div>
  </div>

  <div class="section">Plates (Stepper)</div>
  <div class="grid">
    <div class="rowlabel">Plates</div>
    <button class="open"  onclick="send('plates_open','PLATES OPEN')">OPEN</button>
    <button class="press" onclick="send('plates_press','PLATES PRESS')">PRESS</button>
    <div></div>
  </div>

  <div class="section">Valves</div>
  <div class="grid">
    <div class="rowlabel">Valve 1</div>
    <button class="on"  onclick="send('valve1_open','Valve 1 OPEN')">OPEN</button>
    <button class="off" onclick="send('valve1_close','Valve 1 CLOSE')">CLOSE</button>
    <div></div>

    <div class="rowlabel">Valve 2</div>
    <button class="on"  onclick="send('valve2_open','Valve 2 OPEN')">OPEN</button>
    <button class="off" onclick="send('valve2_close','Valve 2 CLOSE')">CLOSE</button>
    <div></div>

    <div class="rowlabel">Valve 3</div>
    <button class="on"  onclick="send('valve3_open','Valve 3 OPEN')">OPEN</button>
    <button class="off" onclick="send('valve3_close','Valve 3 CLOSE')">CLOSE</button>
    <div></div>

    <div class="rowlabel">Valve 4</div>
    <button class="on"  onclick="send('valve4_open','Valve 4 OPEN')">OPEN</button>
    <button class="off" onclick="send('valve4_close','Valve 4 CLOSE')">CLOSE</button>
    <div></div>

    <div class="rowlabel">Valve 5</div>
    <button class="on"  onclick="send('valve5_open','Valve 5 OPEN')">OPEN</button>
    <button class="off" onclick="send('valve5_close','Valve 5 CLOSE')">CLOSE</button>
    <div></div>
  </div>

  <h2>RPIPLC Analog Output</h2>
  <div class="row">
    <label for="dacval">Set A0.5 (0–4095):</label>
    <input id="dacval" type="number" min="0" max="4095" step="1" value="4095">
    <button onclick="setAnalog()">Set A0.5</button>
  </div>
  <p class="muted">Initialized at server start with <code>rpiplc.init("RPIPLC_V6", "RPIPLC_21")</code> and <code>pin_mode("A0.5", OUTPUT)</code>.</p>

  <div id="log"></div>
</body>
</html>
"""

def hexify(b: bytes) -> str:
    return " ".join(f"0x{v:02X}" for v in b)

def _write_bytes_threadsafe(b: bytes):
    try:
        with SERIAL_LOCK:
            try:
                commands.ser.reset_input_buffer()
                commands.ser.reset_output_buffer()
            except Exception:
                pass
            commands.ser.write(b)
            commands.ser.flush()
    except Exception as e:
        print(f"[write] failed: {e}")

# ---- RPIPLC init on startup (if available) ----
if HAVE_RPIPLC:
    try:
        rpiplc.init("RPIPLC_V6", "RPIPLC_38AR")
        rpiplc.pin_mode("A0.5", rpiplc.OUTPUT)
    except Exception as e:
        HAVE_RPIPLC = False
        RPIPLC_IMPORT_ERR = f"Init error: {e}"

@app.get("/")
def index():
    # render as-is; no Jinja vars used inside PAGE to avoid {{ }} conflicts
    return render_template_string(PAGE)

@app.post("/send")
def send():
    data = request.get_json(silent=True) or {}
    action = data.get("action")

    allowed_special = {"plates_open", "plates_press", "roller_bump"}
    if action not in CMD and action not in allowed_special:
        return jsonify(ok=False, error=f"Unknown action: {action}"), 400

    try:
        if action == "plates_open":
            with SERIAL_LOCK:
                try:
                    commands.ser.reset_input_buffer(); commands.ser.reset_output_buffer()
                except Exception:
                    pass
                commands.write(command_stepper_open)
            return jsonify(ok=True, hex=hexify(command_stepper_open))

        if action == "plates_press":
            with SERIAL_LOCK:
                try:
                    commands.ser.reset_input_buffer(); commands.ser.reset_output_buffer()
                except Exception:
                    pass
                commands.write(command_stepper_press)
            return jsonify(ok=True, hex=hexify(command_stepper_press))

        if action == "roller_bump":
            _write_bytes_threadsafe(CMD["roller_start"])
            threading.Timer(ROLLER_BUMP_SECONDS, lambda: _write_bytes_threadsafe(CMD["roller_stop"])).start()
            return jsonify(ok=True, hex=f"{hexify(CMD['roller_start'])} (auto-stop in {ROLLER_BUMP_SECONDS}s)")

        # Default: raw CMD write
        with SERIAL_LOCK:
            try:
                commands.ser.reset_input_buffer(); commands.ser.reset_output_buffer()
            except Exception:
                pass
            commands.ser.write(CMD[action])
            commands.ser.flush()
        return jsonify(ok=True, hex=hexify(CMD[action]))

    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.post("/home")
def home():
    try:
        with SERIAL_LOCK:
            try:
                commands.ser.reset_input_buffer()
                commands.ser.reset_output_buffer()
            except Exception:
                pass
            with temp_baudrate(commands.ser, HOMING_BAUD):
                commands.write(command_homing1)
                time.sleep(0.50)
                commands.write(command_homing2)
        return jsonify(ok=True, detail=f"{hexify(command_homing1)} then {hexify(command_homing2)}")
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.post("/set_analog")
def set_analog():
    data = request.get_json(silent=True) or {}
    pin = data.get("pin", "A0.5")
    try:
        value = int(data.get("value", 4095))
    except Exception:
        value = 4095
    value = max(0, min(4095, value))  # clamp
    if not HAVE_RPIPLC:
        return jsonify(ok=False, error=f"librpiplc not available or failed to init: {RPIPLC_IMPORT_ERR}"), 500
    try:
        rpiplc.analog_write(pin, value)  # one-shot
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

if __name__ == "__main__":
    # Use a different port if 5000 might be in use; change if you want
    app.run(host="0.0.0.0", port=5091, debug=False)
