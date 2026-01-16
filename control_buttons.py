#!/usr/bin/env python3
# control_buttons.py — Buttons + flow-rate → DAC mapping + STOP for sample filtration

import time
import threading
from flask import Flask, request, jsonify, render_template_string
import commands

# =========================
# RS-485 command bytes (unchanged)
# =========================
command_homing1            = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x00\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\xF8\x65'
command_homing2            = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x02\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\x01\xA2'

command_start_roller       = b'\x01\x06\x00\x07\x01\x00\x39\x9B'
command_stop_roller        = b'\x01\x06\x00\x07\x02\x00\x39\x6B'

command_base_back_on       = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_base_back_off      = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'

command_move_base_front_on  = b'\x01\x06\x00\x08\x01\x00\x09\x98'
command_move_base_front_off = b'\x01\x06\x00\x08\x02\x00\x09\x68'

command_stepper_press      = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x89\x54\x40\xDD\x82'
command_stepper_open       = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x00\x00\x00\x33\x58'

command_open_valve1        = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'
command_open_valve2        = b'\x01\x06\x00\x02\x01\x00\x29\x9A'
command_open_valve3        = b'\x01\x06\x00\x03\x01\x00\x78\x5A'
command_open_valve4        = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_open_valve5        = b'\x01\x06\x00\x05\x01\x00\x98\x5B'

command_start_pump         = b'\x01\x06\x00\x06\x01\x00\x68\x5B'
command_stop_pump          = b'\x01\x06\x00\x06\x02\x00\x68\xAB'

command_close_valve1       = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'
command_close_valve2       = b'\x01\x06\x00\x02\x02\x00\x29\x6A'
command_close_valve3       = b'\x01\x06\x00\x03\x02\x00\x78\xAA'
command_close_valve4       = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'
command_close_valve5       = b'\x01\x06\x00\x05\x02\x00\x98\xAB'

# =========================
# RPIPLC DAC helpers
# =========================
try:
    from librpiplc import rpiplc
    _AO_AVAILABLE = True
except Exception:
    rpiplc = None
    _AO_AVAILABLE = False

AO_PIN = "A0.5"
AO_READY = False

def ao_init():
    global AO_READY
    if not _AO_AVAILABLE:
        print("[AO] librpiplc not available: DAC will be NO-OP.")
        AO_READY = False
        return
    try:
        rpiplc.init("RPIPLC_V6", "RPIPLC_21")
        rpiplc.pin_mode(AO_PIN, rpiplc.OUTPUT)
        AO_READY = True
        print(f"[AO] Ready on {AO_PIN}")
    except Exception as e:
        print(f"[AO] init failed: {e}")
        AO_READY = False

def ao(value: int):
    v = max(0, min(4095, int(value)))
    if AO_READY and _AO_AVAILABLE:
        try:
            rpiplc.analog_write(AO_PIN, v)
        except Exception as e:
            print(f"[AO] write failed: {e}")
    else:
        print(f"[AO] (no-op) {AO_PIN} = {v} (not initialized)")

# =========================
# Flow↔DAC calibration (from your table)
# =========================
FLOW_TO_DAC = {
     5.6:  250,
     8.3:  276,
     9.6:  300,
    20.8:  400,
    32.6:  500,
    39.0:  600,
    54.5:  750,
    76.9: 1000,
   100.0: 1250,
   115.4: 1500,
   117.6: 1750,
   125.0: 2000,
}
FLOW_OPTIONS = sorted(FLOW_TO_DAC.keys())

def dac_for_flow(flow: float) -> int:
    if flow in FLOW_TO_DAC:
        return FLOW_TO_DAC[flow]
    nearest = min(FLOW_TO_DAC.keys(), key=lambda f: abs(f - flow))
    print(f"[AO] Requested {flow} mL/min not calibrated; using nearest {nearest} mL/min")
    return FLOW_TO_DAC[nearest]

# =========================
# Config (current chosen flows)
# =========================
CONFIG = {
    "filtration_flow": 39.0,   # default mL/min
    "cleaning_flow":   100.0,  # default mL/min
}

# =========================
# Flask + threading state
# =========================
app = Flask(__name__)
RUN_LOCK = threading.Lock()
RUNNING = {"active": False, "step": "idle", "last": "", "can_stop": False}

# Event used to stop sample filtration
SAMPLE_STOP = threading.Event()

# =========================
# Functions
# =========================
def initialisation():
    print("Homing 1"); commands.write(command_homing1)
    print("Homing 2"); commands.write(command_homing2)
    time.sleep(120)

def filter_loading():
    print("Start roller (relay 7 ON)"); time.sleep(6)
    print("Stop roller (relay 7 OFF)")
    print("Move base back (relay 4 ON)"); commands.write(command_base_back_on); time.sleep(8)
    print("Stop base movement (relay 4 OFF)"); commands.write(command_base_back_off)
    print("Start plate pressing"); commands.write(command_stepper_press); time.sleep(70)

def sample_filtration():
    """Start pump and wait until Stop is pressed from the UI."""
    flow = CONFIG["filtration_flow"]
    dac  = dac_for_flow(flow)
    print(f"[FILTRATION] Flow {flow} mL/min → DAC {dac}")
    ao(dac)

    print("Open valve 1 (Sample Inlet)");  commands.write(command_open_valve1)
    print("Open valve 2 (Closed Air Inlet)"); commands.write(command_open_valve2)
    print("Open valve 5 (Sample out)");    commands.write(command_open_valve5)

    print("Start peristaltic pump"); commands.write(command_start_pump)

    # set stoppable state and wait for the stop event
    SAMPLE_STOP.clear()
    with RUN_LOCK:
        RUNNING["can_stop"] = True

    print("[FILTRATION] Running… waiting for Stop command")
    try:
        # Wait indefinitely until stop button is pressed
        while not SAMPLE_STOP.wait(timeout=0.2):
            pass
    finally:
        with RUN_LOCK:
            RUNNING["can_stop"] = False

    print("[FILTRATION] Stop received → shutting down")
    # Stop pump and close valves in your order with delays
    print("Stop peristaltic pump"); commands.write(command_stop_pump)
    print("Close valve 1"); commands.write(command_close_valve1); time.sleep(0.3)
    print("Close valve 2"); commands.write(command_close_valve2); time.sleep(0.3)
    print("Close valve 5"); commands.write(command_close_valve5); time.sleep(0.3)

def cleaning():
    flow = CONFIG["cleaning_flow"]
    dac  = dac_for_flow(flow)
    print(f"[CLEANING] Flow {flow} mL/min → DAC {dac}")
    ao(dac)

    # MSF_empty
    commands.write(command_start_pump); time.sleep(120)
    commands.write(command_open_valve3); time.sleep(120)
    commands.write(command_open_valve4); time.sleep(120)
    commands.write(command_stop_pump);  time.sleep(60)
    commands.write(command_close_valve3)
    commands.write(command_close_valve4)



    # MSF_cleaning
    commands.write(command_stepper_open); time.sleep(30)
    commands.write(command_move_base_front_on); time.sleep(10)
    commands.write(command_move_base_front_off)
    commands.write(command_stepper_press); time.sleep(30)
    commands.write(command_open_valve2)

    commands.write(command_start_pump); time.sleep(240)
    commands.write(command_close_valve2); time.sleep(120)
    commands.write(command_stop_pump)
    
# =========================
# Web UI
# =========================
PAGE = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <title>ENVIROMED Quick Controls</title>
  <style>
    body { font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif; max-width: 820px; margin: 24px auto; }
    h1 { margin-bottom: 0.2rem; }
    .status { padding: 12px; border: 1px solid #ddd; border-radius: 8px; margin: 12px 0; }
    button { padding: 10px 16px; border-radius: 8px; border: 1px solid #222; background: #fff; cursor: pointer; margin: 6px 8px 6px 0; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    .row { margin: 12px 0; }
    .small { font-size: 12px; color: #555; }
    select { padding: 8px; border: 1px solid #ccc; border-radius: 6px; }
    .group { padding: 12px; border: 1px dashed #bbb; border-radius: 8px; margin: 14px 0; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
  </style>
</head>
<body>
  <h1>ENVIROMED — Quick Controls</h1>
  <div class="status">
    <div><strong>Status:</strong> <span id="status">loading…</span></div>
    <div class="small">Last: <span id="last">—</span></div>
    <div class="small">Filtration: <span class="mono" id="filFlow">—</span> mL/min → DAC <span class="mono" id="filDac">—</span></div>
    <div class="small">Cleaning:   <span class="mono" id="clnFlow">—</span> mL/min → DAC <span class="mono" id="clnDac">—</span></div>
  </div>

  <div class="group">
    <div class="row">
      <button id="btn-init" onclick="run('initialisation')">Initialisation</button>
      <button id="btn-load" onclick="run('filter_loading')">Filter Loading</button>
      <button id="btn-sample" onclick="run('sample_filtration')">Start Sample Filtration</button>
      <button id="btn-clean" onclick="run('cleaning')">Cleaning</button>
      <button id="btn-stop" style="display:none;background:#fee;" onclick="stopSample()">Stop Filtration</button>
    </div>
  </div>

  <div class="group">
    <div class="row">
      <label>Filtration flow (mL/min):</label>
      <select id="filtration"></select>
      <button onclick="setFlow('filtration', document.getElementById('filtration').value)">Save</button>
    </div>
    <div class="row">
      <label>Cleaning flow (mL/min):</label>
      <select id="cleaning"></select>
      <button onclick="setFlow('cleaning', document.getElementById('cleaning').value)">Save</button>
    </div>
  </div>

<script>
let FLOW_OPTIONS = [];
function refresh() {
  fetch('/status').then(r=>r.json()).then(j=>{
    document.getElementById('status').textContent = j.active ? ('Running: ' + j.step) : 'Idle';
    document.getElementById('last').textContent = j.last || '—';

    // Toggle Stop button only when filtration is running and stoppable
    const showStop = j.active && j.step === 'Sample Filtration' && j.can_stop;
    document.getElementById('btn-stop').style.display = showStop ? 'inline-block' : 'none';

    // Disable all other buttons while any step is running (except Stop)
    ['btn-init','btn-load','btn-sample','btn-clean'].forEach(id=>{
      const b = document.getElementById(id);
      if (b) b.disabled = j.active;
    });
  });
  fetch('/config').then(r=>r.json()).then(j=>{
    document.getElementById('filFlow').textContent = j.filtration_flow.toFixed(1);
    document.getElementById('clnFlow').textContent = j.cleaning_flow.toFixed(1);
    document.getElementById('filDac').textContent = j.filtration_dac;
    document.getElementById('clnDac').textContent = j.cleaning_dac;
    if (FLOW_OPTIONS.length === 0) {
      FLOW_OPTIONS = j.flow_options;
      const make = (id, value) => {
        const el = document.getElementById(id);
        el.innerHTML = '';
        FLOW_OPTIONS.forEach(f=>{
          const opt = document.createElement('option');
          opt.value = f;
          opt.textContent = Number(f).toFixed(1);
          el.appendChild(opt);
        });
        el.value = value;
      };
      make('filtration', j.filtration_flow);
      make('cleaning',   j.cleaning_flow);
    }
  });
}
function setFlow(which, value) {
  fetch('/set_flow', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ which, flow: parseFloat(value) })
  }).then(_=>refresh());
}
function run(step) {
  fetch('/run', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({step})}).then(_=>setTimeout(refresh, 250));
}
function stopSample() {
  fetch('/stop_sample', {method:'POST'}).then(_=>setTimeout(refresh, 250));
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""

def _run_in_thread(fn, label):
    def worker():
        with RUN_LOCK:
            RUNNING["active"] = True
            RUNNING["step"] = label
            RUNNING["last"] = ""
        try:
            fn()
            msg = f"{label} — done"
        except Exception as e:
            msg = f"{label} — ERROR: {e}"
        finally:
            with RUN_LOCK:
                RUNNING["active"] = False
                RUNNING["last"] = msg
                RUNNING["step"] = "idle"
                RUNNING["can_stop"] = False
            print(msg)
    threading.Thread(target=worker, daemon=True).start()

@app.route("/")
def index():
    return render_template_string(PAGE)

@app.route("/status")
def status():
    with RUN_LOCK:
        return jsonify(RUNNING)

@app.route("/config")
def get_config():
    fil_flow = float(CONFIG["filtration_flow"])
    cln_flow = float(CONFIG["cleaning_flow"])
    fil_dac  = dac_for_flow(fil_flow)
    cln_dac  = dac_for_flow(cln_flow)
    return jsonify({
        "flow_options": FLOW_OPTIONS,
        "filtration_flow": fil_flow,
        "cleaning_flow": cln_flow,
        "filtration_dac": fil_dac,
        "cleaning_dac": cln_dac,
    })

@app.route("/set_flow", methods=["POST"])
def set_flow():
    data = request.get_json(force=True)
    which = (data.get("which") or "").strip()
    try:
        flow  = float(data.get("flow"))
    except Exception:
        return jsonify({"ok": False, "error": "invalid flow value"}), 400

    if which == "filtration":
        CONFIG["filtration_flow"] = flow
    elif which == "cleaning":
        CONFIG["cleaning_flow"] = flow
    else:
        return jsonify({"ok": False, "error": "unknown selector"}), 400

    print(f"[CONFIG] {which} flow set to {flow} mL/min → DAC {dac_for_flow(flow)}")
    return jsonify({"ok": True})

@app.route("/run", methods=["POST"])
def run_step():
    data = request.get_json(force=True)
    step = (data.get("step") or "").strip()
    with RUN_LOCK:
        if RUNNING["active"]:
            return jsonify({"ok": False, "error": "Already running"}), 409
    if step == "initialisation":
        _run_in_thread(initialisation, "Initialisation")
    elif step == "filter_loading":
        _run_in_thread(filter_loading, "Filter Loading")
    elif step == "sample_filtration":
        _run_in_thread(sample_filtration, "Sample Filtration")
    elif step == "cleaning":
        _run_in_thread(cleaning, "Cleaning")
    else:
        return jsonify({"ok": False, "error": "Unknown step"}), 400
    return jsonify({"ok": True})

@app.route("/stop_sample", methods=["POST"])
def stop_sample():
    # Trigger stop; sample_filtration loop will exit and perform shutdown/valve closing
    SAMPLE_STOP.set()
    return jsonify({"ok": True, "message": "Stop signal sent"})

if __name__ == "__main__":
    ao_init()
    app.run(host="0.0.0.0", port=5000, debug=False)
