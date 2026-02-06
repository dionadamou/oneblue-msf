#!/usr/bin/env python3
# control_api.py — Flask backend for your current frontend (safe start)
#
# HOMING (WORKING REFERENCE APPLIED):
# - rpiplc.init("RPIPLC_V6", "RPIPLC_38AR")  ✅ (matches your proven snippet)
# - Limit switch on I1.5 by default          ✅
# - Detects HIGH -> LOW -> HIGH
#
# ENV VARS (optional):
#   HOME_LIMIT_PIN=I1.5
#   HOME_TIMEOUT_S=90
#   HOME_POLL_MS=10                # polling interval in ms
#   HOME_STABLE_COUNT=5            # consecutive reads needed to accept a state
#
# Notes:
# - pressed = LOW, released = HIGH (your statement)
# - If you start already LOW, we first wait to see a HIGH (release), then proceed.

import time, threading, os
from flask import Flask, request, jsonify

# -------- Safe import for RS-485 "commands" helper --------
try:
    import commands  # your RS-485 helpers and configured serial port
    HAVE_COMMANDS = True
except Exception as e:
    print(f"[WARN] 'commands' module not available: {e}")
    HAVE_COMMANDS = False

    class _CommandsStub:
        @staticmethod
        def write(payload: bytes):
            print(f"[STUB] commands.write({payload!r})")

    commands = _CommandsStub()

# -------- Frontend build path (adjust if needed) ----------
FRONTEND_DIST = "/home/pi/projects/msf-ui/dist/"

app = Flask(
    __name__,
    static_folder=FRONTEND_DIST,
    static_url_path="/"
)

@app.get("/")
def index():
    return app.send_static_file("index.html")

# =========================
# RS-485 command bytes (unchanged)
# =========================
command_homing1             = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x00\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\xF8\x65'
command_homing2             = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x02\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\x01\xA2'

command_start_roller        = b'\x01\x06\x00\x07\x01\x00\x39\x9B'
command_stop_roller         = b'\x01\x06\x00\x07\x02\x00\x39\x6B'

command_base_back_on        = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_base_back_off       = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'

command_move_base_front_on  = b'\x01\x06\x00\x08\x01\x00\x09\x98'
command_move_base_front_off = b'\x01\x06\x00\x08\x02\x00\x09\x68'

command_stepper_press       = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x72\x70\xE0\xB7\x0B'
command_stepper_open        = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x00\x00\x00\x33\x58'

command_open_valve1         = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'
command_open_valve2         = b'\x01\x06\x00\x02\x01\x00\x29\x9A'
command_open_valve3         = b'\x01\x06\x00\x03\x01\x00\x78\x5A'
command_open_valve4         = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_open_valve5         = b'\x01\x06\x00\x05\x01\x00\x98\x5B'

command_start_pump          = b'\x01\x06\x00\x06\x01\x00\x68\x5B'
command_stop_pump           = b'\x01\x06\x00\x06\x02\x00\x68\xAB'

command_close_valve1        = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'
command_close_valve2        = b'\x01\x06\x00\x02\x02\x00\x29\x6A'
command_close_valve3        = b'\x01\x06\x00\x03\x02\x00\x78\xAA'
command_close_valve4        = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'
command_close_valve5        = b'\x01\x06\x00\x05\x02\x00\x98\xAB'

# =========================
# RPIPLC (AO + digital input)
# =========================
try:
    from librpiplc import rpiplc
    _RPIPLC_AVAILABLE = True
except Exception as e:
    print(f"[WARN] librpiplc not available: {e}")
    rpiplc = None
    _RPIPLC_AVAILABLE = False

AO_PIN   = "A0.5"
AO_READY = False

HOME_LIMIT_PIN = os.environ.get("HOME_LIMIT_PIN", "I1.5").strip()
HOME_TIMEOUT_S = float(os.environ.get("HOME_TIMEOUT_S", "90"))
HOME_POLL_MS = int(os.environ.get("HOME_POLL_MS", "10"))
HOME_STABLE_COUNT = int(os.environ.get("HOME_STABLE_COUNT", "5"))

def rpiplc_init():
    """
    IMPORTANT: Use the same init profile as your proven working digital_read snippet.
    """
    global AO_READY
    if not _RPIPLC_AVAILABLE:
        print("[RPIPLC] Not available: AO + input read disabled.")
        AO_READY = False
        return

    rpiplc.init("RPIPLC_V6", "RPIPLC_38AR")

    # AO pin
    try:
        rpiplc.pin_mode(AO_PIN, rpiplc.OUTPUT)
        AO_READY = True
        print(f"[AO] Ready on {AO_PIN}")
    except Exception as e:
        AO_READY = False
        print(f"[AO] setup failed on {AO_PIN}: {e}")

    # Home input pin
    try:
        rpiplc.pin_mode(HOME_LIMIT_PIN, rpiplc.INPUT)
        print(f"[HOME] Limit input ready: {HOME_LIMIT_PIN}")
    except Exception as e:
        print(f"[HOME] Limit input setup FAILED on {HOME_LIMIT_PIN}: {e}")

def ao(value: int):
    v = max(0, min(4095, int(value)))
    if AO_READY and _RPIPLC_AVAILABLE:
        try:
            rpiplc.analog_write(AO_PIN, v)
        except Exception as e:
            print(f"[AO] write failed: {e}")
    else:
        print(f"[AO] (no-op) {AO_PIN} = {v}")

def _read_pin(pin: str) -> int:
    # raw read like your working snippet
    return 1 if rpiplc.digital_read(pin) else 0

def _wait_stable(pin: str, target: int, stable_count: int, poll_s: float,
                 deadline: float, stop_event: threading.Event = None) -> None:
    consec = 0
    while True:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError("Aborted (STOP_EVENT set)")
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timeout waiting for {pin} stable={target}")

        v = _read_pin(pin)
        if v == target:
            consec += 1
            if consec >= stable_count:
                return
        else:
            consec = 0
        time.sleep(poll_s)

def wait_for_homing_signature(pin: str, timeout_s: float, poll_ms: int,
                              stable_count: int, stop_event: threading.Event = None) -> None:
    """
    Detect: HIGH -> LOW -> HIGH
    """
    poll_s = max(0.001, poll_ms / 1000.0)
    deadline = time.monotonic() + timeout_s

    print(f"[HOME] Watching {pin} for HIGH->LOW->HIGH | poll={poll_ms}ms stable_count={stable_count}")

    # Ensure we're released (HIGH) at least once before looking for the hit.
    # If we start pressed (LOW), we wait for a HIGH first.
    print("[HOME] Stage 0: waiting for HIGH (released)")
    _wait_stable(pin, 1, stable_count, poll_s, deadline, stop_event)

    print("[HOME] Stage 1: waiting for LOW (pressed)")
    _wait_stable(pin, 0, stable_count, poll_s, deadline, stop_event)

    print("[HOME] Stage 2: waiting for HIGH again (released)")
    _wait_stable(pin, 1, stable_count, poll_s, deadline, stop_event)

    print("[HOME] Signature detected → HOMED")

# =========================
# Flow↔DAC calibration (your table)
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
    nearest = min(FLOW_TO_DAC, key=lambda f: abs(f - flow))
    print(f".[AO] {flow} not calibrated → using {nearest}")
    return FLOW_TO_DAC[nearest]

# =========================
# Sequences
# =========================
def initialisation(stop_event: threading.Event):
    print("Homing 1"); commands.write(command_homing1)
    print("Homing 2"); commands.write(command_homing2)

    wait_for_homing_signature(
        pin=HOME_LIMIT_PIN,
        timeout_s=HOME_TIMEOUT_S,
        poll_ms=HOME_POLL_MS,
        stable_count=HOME_STABLE_COUNT,
        stop_event=stop_event
    )

    print("[HOME] Initialisation complete (homed)")

def filter_loading():
    print("Start roller (relay 7 ON)"); commands.write(command_start_roller)
    time.sleep(6)
    print("Stop roller (relay 7 OFF)"); commands.write(command_stop_roller)

    print("Move base back (relay 4 ON)"); commands.write(command_base_back_on); time.sleep(10)
    print("Stop base movement (relay 4 OFF)"); commands.write(command_base_back_off)
    print("Start plate pressing"); commands.write(command_stepper_press); time.sleep(70)

def sample_filtration(flow: float, stop_event: threading.Event):
    dac = dac_for_flow(flow)
    print(f"[FILTRATION] {flow} mL/min → DAC {dac}"); ao(dac)

    print("Open valve 1"); commands.write(command_open_valve1)
    print("Open valve 2"); commands.write(command_open_valve2)
    print("Open valve 5"); commands.write(command_open_valve5)

    print("Start pump"); commands.write(command_start_pump)
    print("[FILTRATION] Waiting for Stop…")
    try:
        while not stop_event.wait(timeout=0.2):
            pass
    finally:
        print("[FILTRATION] Stopping…")
        print("Stop pump"); commands.write(command_stop_pump)
        print("Close valve 1"); commands.write(command_close_valve1); time.sleep(0.3)
        print("Close valve 2"); commands.write(command_close_valve2); time.sleep(0.3)
        print("Close valve 5"); commands.write(command_close_valve5); time.sleep(0.3)

def cleaning(flow: float):
    dac = dac_for_flow(flow)
    print(f"[CLEANING] {flow} mL/min → DAC {dac}"); ao(dac)

    commands.write(command_close_valve1); time.sleep(0.3)
    commands.write(command_close_valve2); time.sleep(0.3)
    commands.write(command_close_valve5); time.sleep(0.3)

    # MSF_empty
    commands.write(command_start_pump); time.sleep(30)
    commands.write(command_open_valve3); time.sleep(30)
    commands.write(command_open_valve4); time.sleep(30)
    commands.write(command_stop_pump);  time.sleep(30)
    commands.write(command_close_valve3)
    commands.write(command_close_valve4)

    # MSF_cleaning
    commands.write(command_stepper_open); time.sleep(30)
    commands.write(command_start_roller); time.sleep(2)
    commands.write(command_stop_roller)
    commands.write(command_move_base_front_on); time.sleep(10)
    commands.write(command_move_base_front_off)
    commands.write(command_stepper_press); time.sleep(30)
    commands.write(command_open_valve2)

    commands.write(command_start_pump); time.sleep(60)
    commands.write(command_close_valve2); time.sleep(40)
    commands.write(command_stop_pump)

# =========================
# App state + API
# =========================
RUN_LOCK = threading.Lock()
RUNNING = {"active": False, "step": "idle", "last": "", "can_stop": False}
STOP_EVENT = threading.Event()

CONFIG = {"filtration_flow": 39.0, "cleaning_flow": 100.0}

def _run_async(fn, label):
    def worker():
        with RUN_LOCK:
            RUNNING.update(active=True, step=label, last="", can_stop=(label == "Sample Filtration"))
        try:
            fn()
            msg = f"{label} — done"
        except Exception as e:
            msg = f"{label} — ERROR: {e}"
        finally:
            with RUN_LOCK:
                RUNNING.update(active=False, step="idle", last=msg, can_stop=False)
            print(msg)
    threading.Thread(target=worker, daemon=True).start()

@app.get("/status")
def get_status():
    with RUN_LOCK:
        return jsonify(RUNNING)

@app.get("/config")
def get_config():
    fil = float(CONFIG["filtration_flow"]); cln = float(CONFIG["cleaning_flow"])
    return jsonify({
        "flow_options": FLOW_OPTIONS,
        "filtration_flow": fil,
        "cleaning_flow": cln,
        "filtration_dac": dac_for_flow(fil),
        "cleaning_dac": dac_for_flow(cln),
        "home_limit_pin": HOME_LIMIT_PIN,
        "home_timeout_s": HOME_TIMEOUT_S,
        "home_poll_ms": HOME_POLL_MS,
        "home_stable_count": HOME_STABLE_COUNT,
        "rpiplc_profile": "RPIPLC_V6 / RPIPLC_38AR",
    })

@app.post("/set_flow")
def set_flow():
    data = request.get_json(force=True) or {}
    which = (data.get("which") or "").strip()
    try:
        flow = float(data["flow"])
    except Exception:
        return jsonify({"ok": False, "error": "invalid flow"}), 400

    if which == "filtration":
        CONFIG["filtration_flow"] = flow
    elif which == "cleaning":
        CONFIG["cleaning_flow"] = flow
    else:
        return jsonify({"ok": False, "error": "unknown selector"}), 400

    print(f"[CONFIG] {which} → {flow} mL/min (DAC {dac_for_flow(flow)})")
    return jsonify({"ok": True})

@app.post("/run")
def run_step():
    try:
        data = request.get_json(force=True) or {}
    except Exception as e:
        return jsonify({"ok": False, "error": f"Invalid JSON: {e}"}), 400

    step = (data.get("step") or "").strip()

    with RUN_LOCK:
        if RUNNING["active"]:
            return jsonify({"ok": False, "error": "Already running"}), 409

    if step == "initialisation":
        STOP_EVENT.clear()
        _run_async(lambda: initialisation(STOP_EVENT), "Initialisation")
    elif step == "filter_loading":
        _run_async(filter_loading, "Filter Loading")
    elif step == "sample_filtration":
        STOP_EVENT.clear()
        _run_async(lambda: sample_filtration(CONFIG["filtration_flow"], STOP_EVENT),
                   "Sample Filtration")
    elif step == "cleaning":
        _run_async(lambda: cleaning(CONFIG["cleaning_flow"]), "Cleaning")
    else:
        return jsonify({"ok": False, "error": "Unknown step"}), 400

    return jsonify({"ok": True})

@app.post("/stop_sample")
def stop_sample():
    STOP_EVENT.set()
    return jsonify({"ok": True, "message": "Stop signal sent"})

@app.get("/debug/input/<pin>")
def debug_input(pin):
    if rpiplc is None or not _RPIPLC_AVAILABLE:
        return jsonify({"ok": False, "error": "rpiplc not available"}), 500
    try:
        val = 1 if rpiplc.digital_read(pin) else 0
        return jsonify({"ok": True, "pin": pin, "value": val})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

if __name__ == "__main__":
    print(f"[INFO] Static dist: {FRONTEND_DIST} (index exists: {os.path.exists(os.path.join(FRONTEND_DIST,'index.html'))})")
    rpiplc_init()
    print(f"[INFO] Home limit: {HOME_LIMIT_PIN} timeout={HOME_TIMEOUT_S}s poll={HOME_POLL_MS}ms stable_count={HOME_STABLE_COUNT}")
    app.run(host="0.0.0.0", port=5010, debug=False)
