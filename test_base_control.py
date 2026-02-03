#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ENVIROMED Quick Controls (RS-485) + RPIPLC analog + Plates/Homing + Roller
# + SLF3x/SCC1 flow meter integration + volume-based auto-stop
# + Flowmeter power relay (RPIPLC R1.2) ON/OFF/CYCLE buttons

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

# -------------------------
# RPIPLC (optional)
# -------------------------
try:
    from librpiplc import rpiplc
    HAVE_RPIPLC = True
    RPIPLC_IMPORT_ERR = ""
except Exception as e:
    rpiplc = None
    HAVE_RPIPLC = False
    RPIPLC_IMPORT_ERR = str(e)

RPIPLC_BOARD = "RPIPLC_V6"
RPIPLC_MODEL = "RPIPLC_38AR"

# Flowmeter power relay (same as your logger)
FLOW_PWR_RELAY_CH = "R1.2"   # SCC1 power relay
FLOW_PWR_ASSUME_HIGH_ON = True  # HIGH = ON like your script

# Analog output pin (your existing)
AO_PIN = "A0.5"

# -------------------------
# Flow sensor imports (SLF3x over SCC1 / SHDLC)
# -------------------------
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_driver.errors import ShdlcDeviceError, ShdlcTimeoutError
from sensirion_uart_scc1.drivers.scc1_slf3x import Scc1Slf3x
from sensirion_uart_scc1.drivers.slf_common import get_flow_unit_label
from sensirion_uart_scc1.scc1_shdlc_device import Scc1ShdlcDevice

# Optional RS485 RTS/DE control via pyserial (usually OFF on PLC ports)
try:
    from serial.rs485 import RS485Settings
except Exception:
    RS485Settings = None

# -------------------------
# CONFIG: Flow + Volume Run
# -------------------------
FLOW_PORT = "/dev/ttySC3"
FLOW_ADDR = 0
FLOW_INTERVAL_MS = 2

TARGET_ML_DEFAULT = 20.0
START_THRESHOLD_ML_MIN_DEFAULT = 0.5
STATUS_POLL_MS = 100  # JS poll interval (100 ms)

FLOW_LOCK = threading.Lock()
FLOW_STATE = {
    "ok": False,
    "ml_min": 0.0,
    "temp_c": 0.0,
    "unit": "mL/min",
    "last_ts": 0.0,
    "err": "",
}

VOLUME_LOCK = threading.Lock()
VOLUME_RUN = {
    "active": False,
    "target_ml": TARGET_ML_DEFAULT,
    "start_threshold_ml_min": START_THRESHOLD_ML_MIN_DEFAULT,
    "delivered_ml": 0.0,
    "started_integrating": False,
    "start_ts": 0.0,
    "stop_reason": "",
}
STOP_VOLUME_EVENT = threading.Event()

PWR_LOCK = threading.Lock()
POWER_STATE = {
    "relay_ok": False,   # RPIPLC available + pin configured
    "flow_power_on": None,  # True/False/None(unknown)
    "err": "",
}

# -------------------------
# RS-485 command bytes
# -------------------------
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

# Roller bump duration
ROLLER_BUMP_SECONDS = 6

app = Flask(__name__)

PAGE = """
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>ENVIROMED Quick Controls — Roller + Flow</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
  body { font-family: system-ui, sans-serif; margin: 20px; }
  h1 { margin-bottom: 12px; }
  .grid { display: grid; grid-template-columns: 1fr auto auto auto; gap: 8px 12px; max-width: 1200px; }
  .rowlabel { padding: 10px 0; }
  button { padding: 8px 14px; border: 0; border-radius: 10px; cursor: pointer; }
  .on { background: #e6f4ea; }
  .off { background: #fdeaea; }
  .start { background: #e6f0ff; }
  .stop { background: #fff0f0; }
  .home { background: #fff7e6; }
  .press { background: #ffe6f0; }
  .open { background: #e6fff3; }
  .pill { display:inline-block; padding:6px 10px; border-radius: 999px; border:1px solid #ddd; min-width:90px; text-align:center; font-variant-numeric: tabular-nums; background: #fff; }
  .pillwide { min-width: 160px; }
  #log { margin-top: 16px; white-space: pre-wrap; background:#f6f6f6; padding:10px; border-radius:10px; max-width: 1200px; min-height: 160px; height: 260px; overflow:auto;}
  .row { display:flex; align-items:center; gap:8px; flex-wrap: wrap; }
  input[type="number"] { width: 110px; padding:6px 8px; border-radius:8px; border:1px solid #ddd; }
  .muted { color:#666; font-size: 0.9rem; }
  .section { margin-top: 18px; font-weight: 600; }
</style>

<script>
let pumpTimerInterval = null;
let pumpStartEpoch = null;

let lastAutoStopReason = "";
let lastVolumeActive = false;

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

function logLine(msg){
  const ts = new Date().toLocaleTimeString();
  const line = `[${ts}] ${msg}\\n`;
  const log = document.getElementById("log");
  log.textContent += line;
  log.scrollTop = log.scrollHeight;
}

async function send(action, label){
  const r = await fetch("/send", {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body: JSON.stringify({action})
  });
  const j = await r.json();
  logLine(`${j.ok ? "OK" : "ERR"} ${label}  ${j.hex || ""}  ${j.error || ""}`);

  if (j.ok && action === "pump_start"){
    startPumpTimer();
    lastAutoStopReason = "";
  }
  if (j.ok && action === "pump_stop"){
    stopPumpTimer();
  }
}

async function setAnalog(){
  const val = document.getElementById("dacval").value;
  const r = await fetch("/set_analog", {
    method:"POST",
    headers:{"Content-Type":"application/json"},
    body: JSON.stringify({pin:"A0.5", value: Number(val)})
  });
  const j = await r.json();
  logLine(`${j.ok ? "OK" : "ERR"} Set A0.5 = ${val}  ${j.error || ""}`);
}

async function home(){
  const r = await fetch("/home", {method:"POST"});
  const j = await r.json();
  logLine(`${j.ok ? "OK" : "ERR"} Home  ${j.detail || ""} ${j.error || ""}`);
}

function fmtNum(x, digits=2){
  if (x === null || x === undefined || Number.isNaN(Number(x))) return "--";
  return Number(x).toFixed(digits);
}

async function pollStatus(){
  try{
    const r = await fetch("/status", {cache:"no-store"});
    const s = await r.json();

    // Flow pills
    const flowOkEl = document.getElementById("flowOk");
    const flowEl = document.getElementById("flowPill");
    const tempEl = document.getElementById("tempPill");
    const volEl  = document.getElementById("volPill");
    const tgtEl  = document.getElementById("tgtPill");
    const stEl   = document.getElementById("startThPill");
    const stateEl= document.getElementById("runStatePill");
    const flowErrEl = document.getElementById("flowErrText");

    const pwrEl = document.getElementById("pwrPill");

    if (s.flow_ok){
      flowOkEl.textContent = "FLOW: OK";
      flowOkEl.style.background = "#e6f4ea";
      flowErrEl.textContent = "";
    } else {
      flowOkEl.textContent = "FLOW: ERR";
      flowOkEl.style.background = "#fdeaea";
      flowErrEl.textContent = s.flow_err ? ("Error: " + s.flow_err) : "";
    }

    flowEl.textContent = `${fmtNum(s.flow_ml_min, 2)} mL/min`;
    tempEl.textContent = `${fmtNum(s.temp_c, 1)} C`;

    const vr = s.volume_run || {};
    const active = !!vr.active;
    const delivered = Number(vr.delivered_ml || 0);
    const target = Number(vr.target_ml || 0);
    const started = !!vr.started_integrating;
    const stopReason = String(vr.stop_reason || "");

    volEl.textContent = `${fmtNum(delivered, 2)} mL`;
    tgtEl.textContent = `Target ${fmtNum(target, 0)} mL`;
    stEl.textContent  = `Start@ ${fmtNum(vr.start_threshold_ml_min || 0, 1)} mL/min`;

    if (active){
      stateEl.textContent = started ? "RUN: integrating" : "RUN: waiting flow";
      stateEl.style.background = "#e6f0ff";
    } else {
      stateEl.textContent = stopReason ? `RUN: stopped (${stopReason})` : "RUN: idle";
      stateEl.style.background = "#f1f1f1";
    }

    // Power relay pill
    if (s.power && s.power.relay_ok){
      const on = s.power.flow_power_on;
      if (on === true){
        pwrEl.textContent = "SCC1 PWR: ON";
        pwrEl.style.background = "#e6f4ea";
      } else if (on === false){
        pwrEl.textContent = "SCC1 PWR: OFF";
        pwrEl.style.background = "#fdeaea";
      } else {
        pwrEl.textContent = "SCC1 PWR: ?";
        pwrEl.style.background = "#f1f1f1";
      }
    } else {
      pwrEl.textContent = "SCC1 PWR: N/A";
      pwrEl.style.background = "#f1f1f1";
    }

    // Auto-stop behavior: stop timer + log once
    if (lastVolumeActive && !active && stopReason && stopReason.includes("target reached")){
      if (pumpTimerInterval){
        stopPumpTimer();
      }
      if (lastAutoStopReason !== stopReason){
        lastAutoStopReason = stopReason;
        logLine(`OK Pump AUTO-STOP  (${stopReason})`);
      }
    }
    lastVolumeActive = active;

  }catch(e){
    // ignore occasional fetch errors
  }
}

window.addEventListener("load", ()=>{
  setInterval(pollStatus, %POLL_MS%);
  pollStatus();
});
</script>
</head>
<body>
  <h1>ENVIROMED — Quick Controls</h1>
  <p class="muted">RS-485 bytes via <code>commands.ser.write(...)</code> & <code>commands.write(...)</code> • RPIPLC analog output via <code>librpiplc</code> • Flow auto-stop: integrate after flow ≥ threshold, stop at target mL</p>

  <div class="section">Movement & Pump</div>
  <div class="grid">
    <div class="rowlabel">Move Base Front</div>
    <button class="on"  onclick="send('move_base_front_on','Move Base Front ON')">ON</button>
    <button class="off" onclick="send('move_base_front_off','Move Base Front OFF')">OFF</button>
    <div></div>

    <div class="rowlabel">Peristaltic Pump</div>
    <button class="start" onclick="send('pump_start','Pump START (auto 20mL)')">START</button>
    <button class="stop"  onclick="send('pump_stop','Pump STOP')">STOP</button>
    <span id="pumpTimer" class="pill">00:00:00</span>

    <div class="rowlabel">Flow / Volume</div>
    <span id="flowOk" class="pill pillwide">FLOW: --</span>
    <span id="flowPill" class="pill pillwide">-- mL/min</span>
    <span id="tempPill" class="pill">-- C</span>

    <div class="rowlabel">Volume Run</div>
    <span id="runStatePill" class="pill pillwide">RUN: --</span>
    <span id="volPill" class="pill pillwide">-- mL</span>
    <span id="tgtPill" class="pill pillwide">Target -- mL</span>

    <div class="rowlabel">Start Threshold</div>
    <span id="startThPill" class="pill pillwide">Start@ -- mL/min</span>
    <div></div>
    <div></div>

    <div class="rowlabel">SCC1 / Flowmeter Power</div>
    <button class="on"  onclick="send('flow_power_on','Flowmeter Power ON')">POWER ON</button>
    <button class="off" onclick="send('flow_power_off','Flowmeter Power OFF')">POWER OFF</button>
    <button class="home" onclick="send('flow_power_cycle','Flowmeter Power CYCLE')">POWER CYCLE</button>

    <div class="rowlabel">Power Status</div>
    <span id="pwrPill" class="pill pillwide">SCC1 PWR: --</span>
    <div></div>
    <div></div>

    <div class="rowlabel">Flow status detail</div>
    <span id="flowErrText" class="muted"></span>
    <div></div>
    <div></div>

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
  <p class="muted">Initialized at server start with <code>rpiplc.init("RPIPLC_V6", "RPIPLC_38AR")</code> and <code>pin_mode("A0.5", OUTPUT)</code>.</p>

  <div id="log"></div>
</body>
</html>
"""

def _page_with_poll_ms(page: str, poll_ms: int) -> str:
    return page.replace("%POLL_MS%", str(int(poll_ms)))

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

# -------------------------
# RPIPLC init + pins
# -------------------------
def rpiplc_init_once():
    global HAVE_RPIPLC, RPIPLC_IMPORT_ERR
    if not HAVE_RPIPLC:
        with PWR_LOCK:
            POWER_STATE["relay_ok"] = False
            POWER_STATE["flow_power_on"] = None
            POWER_STATE["err"] = RPIPLC_IMPORT_ERR
        return

    try:
        rpiplc.init(RPIPLC_BOARD, RPIPLC_MODEL)

        # Analog pin (optional)
        try:
            rpiplc.pin_mode(AO_PIN, rpiplc.OUTPUT)
        except Exception:
            pass

        # Flow power relay pin
        rpiplc.pin_mode(FLOW_PWR_RELAY_CH, rpiplc.OUTPUT)

        with PWR_LOCK:
            POWER_STATE["relay_ok"] = True
            POWER_STATE["err"] = ""
            # we don't know actual state; set to "assume ON" after we command it
            POWER_STATE["flow_power_on"] = None

    except Exception as e:
        HAVE_RPIPLC = False
        RPIPLC_IMPORT_ERR = f"Init error: {e}"
        with PWR_LOCK:
            POWER_STATE["relay_ok"] = False
            POWER_STATE["flow_power_on"] = None
            POWER_STATE["err"] = RPIPLC_IMPORT_ERR

def flow_power_on():
    if not HAVE_RPIPLC:
        raise RuntimeError(f"RPIPLC unavailable: {RPIPLC_IMPORT_ERR}")
    rpiplc.digital_write(FLOW_PWR_RELAY_CH, rpiplc.HIGH if FLOW_PWR_ASSUME_HIGH_ON else rpiplc.LOW)
    with PWR_LOCK:
        POWER_STATE["flow_power_on"] = True

def flow_power_off():
    if not HAVE_RPIPLC:
        raise RuntimeError(f"RPIPLC unavailable: {RPIPLC_IMPORT_ERR}")
    rpiplc.digital_write(FLOW_PWR_RELAY_CH, rpiplc.LOW if FLOW_PWR_ASSUME_HIGH_ON else rpiplc.HIGH)
    with PWR_LOCK:
        POWER_STATE["flow_power_on"] = False

def flow_power_cycle(off_ms=1500, on_ms=2500):
    flow_power_off()
    rpiplc.delay(int(off_ms))
    flow_power_on()
    rpiplc.delay(int(on_ms))

# Init RPIPLC on startup (if present)
rpiplc_init_once()

# Optional: ensure SCC1 is powered ON at server start
if HAVE_RPIPLC:
    try:
        flow_power_on()
        rpiplc.delay(200)
    except Exception:
        pass

# -------------------------
# Flow sensor thread
# -------------------------
def enable_rs485_mode(shdlc_port):
    if RS485Settings is None:
        return
    candidates = ["_serial", "serial", "_ser", "_port", "port"]
    ser = None
    for name in candidates:
        obj = getattr(shdlc_port, name, None)
        if obj is not None and hasattr(obj, "rs485_mode"):
            ser = obj
            break
    if ser is None:
        return

    ser.rs485_mode = RS485Settings(
        rts_level_for_tx=True,
        rts_level_for_rx=False,
        delay_before_tx=0.0,
        delay_before_rx=0.0,
    )
    if hasattr(ser, "rts"):
        ser.rts = False

def init_slf3x_sensor(serial_port: str, address: int, interval_ms: int):
    port = ShdlcSerialPort(port=serial_port, baudrate=115200)
    # If you ever need RTS direction control, uncomment:
    # enable_rs485_mode(port)

    device = Scc1ShdlcDevice(ShdlcConnection(port), slave_address=address)
    sensor = Scc1Slf3x(device)

    _ = sensor.serial_number
    _ = sensor.product_id

    flow_scale, unit = sensor.get_flow_unit_and_scale()
    unit_label = get_flow_unit_label(unit)

    sensor.start_continuous_measurement(interval_ms=interval_ms)
    return port, sensor, flow_scale, unit_label

def flow_reader_thread():
    while True:
        port = None
        sensor = None
        try:
            port, sensor, flow_scale, unit_label = init_slf3x_sensor(
                serial_port=FLOW_PORT,
                address=FLOW_ADDR,
                interval_ms=FLOW_INTERVAL_MS,
            )
            with FLOW_LOCK:
                FLOW_STATE["ok"] = True
                FLOW_STATE["unit"] = unit_label  # you said this is mL/min
                FLOW_STATE["err"] = ""

            while True:
                remaining, lost, data = sensor.read_extended_buffer()
                if not data:
                    continue

                flow_vals = [(f / flow_scale) for (f, temp, flag) in data]
                temp_vals = [(temp / 200.0) for (f, temp, flag) in data]
                flow_val = sum(flow_vals) / len(flow_vals)
                temp_c = sum(temp_vals) / len(temp_vals)
                

                with FLOW_LOCK:
                    FLOW_STATE["ml_min"] = float(flow_val)
                    FLOW_STATE["temp_c"] = float(temp_c)
                    FLOW_STATE["last_ts"] = time.time()
                    FLOW_STATE["ok"] = True

        except (ShdlcDeviceError, ShdlcTimeoutError) as e:
            with FLOW_LOCK:
                FLOW_STATE["ok"] = False
                FLOW_STATE["err"] = f"SHDLC error: {e}"
            time.sleep(1.0)

        except Exception as e:
            with FLOW_LOCK:
                FLOW_STATE["ok"] = False
                FLOW_STATE["err"] = f"Flow thread error: {e}"
            time.sleep(1.0)

        finally:
            try:
                if sensor is not None:
                    sensor.stop_continuous_measurement()
                    time.sleep(0.2)
            except Exception:
                pass
            try:
                if port is not None:
                    port.close()
            except Exception:
                pass
            time.sleep(0.5)

def get_flow_ml_min() -> float:
    with FLOW_LOCK:
        f = float(FLOW_STATE.get("ml_min", 0.0) or 0.0)
    return max(0.0, f)

threading.Thread(target=flow_reader_thread, daemon=True).start()

# -------------------------
# Volume-run auto-stop thread
# -------------------------
def volume_run_thread(target_ml: float, start_threshold_ml_min: float):
    with VOLUME_LOCK:
        VOLUME_RUN["active"] = True
        VOLUME_RUN["target_ml"] = float(target_ml)
        VOLUME_RUN["start_threshold_ml_min"] = float(start_threshold_ml_min)
        VOLUME_RUN["delivered_ml"] = 0.0
        VOLUME_RUN["started_integrating"] = False
        VOLUME_RUN["start_ts"] = time.time()
        VOLUME_RUN["stop_reason"] = ""

    STOP_VOLUME_EVENT.clear()

    delivered = 0.0
    t_prev = time.monotonic()

    try:
        while not STOP_VOLUME_EVENT.wait(0.05):
            flow_now = get_flow_ml_min()
            now = time.monotonic()

            with VOLUME_LOCK:
                started = bool(VOLUME_RUN["started_integrating"])
                tgt = float(VOLUME_RUN["target_ml"])
                thr = float(VOLUME_RUN["start_threshold_ml_min"])

            if not started:
                if flow_now >= thr:
                    with VOLUME_LOCK:
                        VOLUME_RUN["started_integrating"] = True
                    t_prev = now
                else:
                    t_prev = now
                    continue

            dt = now - t_prev
            t_prev = now

            delivered += (flow_now * dt) / 60.0

            with VOLUME_LOCK:
                VOLUME_RUN["delivered_ml"] = delivered

            if delivered >= tgt:
                try:
                    _write_bytes_threadsafe(CMD["pump_stop"])
                except Exception:
                    pass
                with VOLUME_LOCK:
                    VOLUME_RUN["stop_reason"] = f"target reached ({delivered:.2f} mL)"
                break

    finally:
        with VOLUME_LOCK:
            VOLUME_RUN["active"] = False
        STOP_VOLUME_EVENT.clear()

# -------------------------
# Flask routes
# -------------------------
@app.get("/")
def index():
    return render_template_string(_page_with_poll_ms(PAGE, STATUS_POLL_MS))

@app.post("/send")
def send():
    data = request.get_json(silent=True) or {}
    action = data.get("action")

    allowed_special = {
        "plates_open", "plates_press", "roller_bump",
        "flow_power_on", "flow_power_off", "flow_power_cycle",
    }
    if action not in CMD and action not in allowed_special:
        return jsonify(ok=False, error=f"Unknown action: {action}"), 400

    try:
        # ---- Flow power relay controls ----
        if action == "flow_power_on":
            flow_power_on()
            return jsonify(ok=True, hex=f"{FLOW_PWR_RELAY_CH}=ON")

        if action == "flow_power_off":
            flow_power_off()
            return jsonify(ok=True, hex=f"{FLOW_PWR_RELAY_CH}=OFF")

        if action == "flow_power_cycle":
            if not HAVE_RPIPLC:
                return jsonify(ok=False, error=f"RPIPLC unavailable: {RPIPLC_IMPORT_ERR}"), 500
            flow_power_cycle(off_ms=1500, on_ms=2500)
            return jsonify(ok=True, hex=f"{FLOW_PWR_RELAY_CH}=CYCLE (1500ms off, 2500ms on)")

        # ---- Existing special actions ----
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

        # ---- Pump start/stop with volume auto-stop ----
        if action == "pump_start":
            with SERIAL_LOCK:
                try:
                    commands.ser.reset_input_buffer(); commands.ser.reset_output_buffer()
                except Exception:
                    pass
                commands.ser.write(CMD["pump_start"])
                commands.ser.flush()

            # Cancel any previous volume run and start a new one
            STOP_VOLUME_EVENT.set()
            time.sleep(0.05)
            STOP_VOLUME_EVENT.clear()

            with VOLUME_LOCK:
                VOLUME_RUN["stop_reason"] = ""

            threading.Thread(
                target=volume_run_thread,
                args=(TARGET_ML_DEFAULT, START_THRESHOLD_ML_MIN_DEFAULT),
                daemon=True
            ).start()

            return jsonify(ok=True, hex=hexify(CMD["pump_start"]) + f" (auto-stop @ {int(TARGET_ML_DEFAULT)} mL)")

        if action == "pump_stop":
            STOP_VOLUME_EVENT.set()
            with SERIAL_LOCK:
                try:
                    commands.ser.reset_input_buffer(); commands.ser.reset_output_buffer()
                except Exception:
                    pass
                commands.ser.write(CMD["pump_stop"])
                commands.ser.flush()

            with VOLUME_LOCK:
                VOLUME_RUN["stop_reason"] = "manual stop"

            return jsonify(ok=True, hex=hexify(CMD["pump_stop"]))

        # ---- Default raw CMD write ----
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
    pin = data.get("pin", AO_PIN)
    try:
        value = int(data.get("value", 4095))
    except Exception:
        value = 4095
    value = max(0, min(4095, value))
    if not HAVE_RPIPLC:
        return jsonify(ok=False, error=f"librpiplc not available or failed to init: {RPIPLC_IMPORT_ERR}"), 500
    try:
        rpiplc.analog_write(pin, value)
        return jsonify(ok=True)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

@app.get("/status")
def status():
    with FLOW_LOCK:
        flow_ok = bool(FLOW_STATE["ok"])
        flow_ml_min = float(FLOW_STATE["ml_min"])
        temp_c = float(FLOW_STATE["temp_c"])
        flow_err = str(FLOW_STATE["err"])
        last_ts = float(FLOW_STATE["last_ts"])
        unit = str(FLOW_STATE["unit"])

    with VOLUME_LOCK:
        vr = dict(VOLUME_RUN)

    with PWR_LOCK:
        pwr = dict(POWER_STATE)

    return jsonify(
        flow_ok=flow_ok,
        flow_ml_min=flow_ml_min,
        temp_c=temp_c,
        flow_err=flow_err,
        flow_last_ts=last_ts,
        flow_unit=unit,
        volume_run=vr,
        power=pwr,
    )

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5091, debug=False)
