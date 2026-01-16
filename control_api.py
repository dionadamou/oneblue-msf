#!/usr/bin/env python3
"""
control_api.py — API-only controller for MSF device (OneBlue MSF)

Drop-in version with:
- High-level steps + low-level actions
- SSE events + optional webhook/MQTT notifications
- RPIPLC AO (optional; disabled if DISABLE_FLOW=1 or missing)
- Master power relay enable on RPIPLC DO (R1.3 by default)
    - HIGH  = machine powered
    - LOW   = machine power cut (E-STOP)
- Boot default: master power HIGH (configurable)

IMPORTANT NOTE:
Your current issue is very likely because rpiplc.init() was called twice with
two different profiles (RPIPLC_21 for AO and RPIPLC_38AR for DO). Many stacks
don’t like that.

This script initialises RPIPLC ONCE (single profile), then configures both AO + DO.
Default profile is "RPIPLC_38AR" (because your DO test worked with that).
"""

import os, sys, time, json, uuid, hmac, hashlib, threading
from functools import wraps
from typing import Dict, Any
from flask import Flask, request, jsonify, Response

sys.path.insert(0, "/home/pi/maria")

# Optional libraries
try:
    import requests as _requests
except Exception:
    _requests = None

try:
    import paho.mqtt.client as mqtt
    _MQTT_AVAILABLE = True
except Exception:
    mqtt = None
    _MQTT_AVAILABLE = False

# --------------------------------------------------------
# RS485 Serial Commands (via your 'commands' module)
# --------------------------------------------------------
try:
    import commands
    HAVE_COMMANDS = True
except Exception as e:
    print(f"[WARN] 'commands' module not available: {e}")
    HAVE_COMMANDS = False

    class _CmdStub:
        @staticmethod
        def write(payload: bytes):
            print(f"[STUB] commands.write({payload!r})")

    commands = _CmdStub()

# --------------------------------------------------------
# RS485 Command Bytes
# --------------------------------------------------------
command_homing1 = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x00\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\xF8\x65'
command_homing2 = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x07\x00\x02\x03\x01\xF4\x00\x00\x03\xE8\x00\x00\x27\x10\x01\xA2'

command_start_roller = b'\x01\x06\x00\x07\x01\x00\x39\x9B'
command_stop_roller  = b'\x01\x06\x00\x07\x02\x00\x39\x6B'

command_base_back_on  = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_base_back_off = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'

command_move_base_front_on  = b'\x01\x06\x00\x08\x01\x00\x09\x98'
command_move_base_front_off = b'\x01\x06\x00\x08\x02\x00\x09\x68'

command_stepper_press = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x72\x70\xE0\xB7\x0B'
command_stepper_open  = b'\x4E\x10\xA7\x9E\x00\x07\x0E\x01\x00\x00\x03\x03\xE8\x00\x00\x4E\x20\x00\x00\x00\x00\x33\x58'

command_open_valve1 = b'\x01\x06\x00\x01\x01\x00\xD9\x9A'
command_open_valve2 = b'\x01\x06\x00\x02\x01\x00\x29\x9A'
command_open_valve3 = b'\x01\x06\x00\x03\x01\x00\x78\x5A'
command_open_valve4 = b'\x01\x06\x00\x04\x01\x00\xC9\x9B'
command_open_valve5 = b'\x01\x06\x00\x05\x01\x00\x98\x5B'

command_close_valve1 = b'\x01\x06\x00\x01\x02\x00\xD9\x6A'
command_close_valve2 = b'\x01\x06\x00\x02\x02\x00\x29\x6A'
command_close_valve3 = b'\x01\x06\x00\x03\x02\x00\x78\xAA'
command_close_valve4 = b'\x01\x06\x00\x04\x02\x00\xC9\x6B'
command_close_valve5 = b'\x01\x06\x00\x05\x02\x00\x98\xAB'

command_start_pump = b'\x01\x06\x00\x06\x01\x00\x68\x5B'
command_stop_pump  = b'\x01\x06\x00\x06\x02\x00\x68\xAB'

# --------------------------------------------------------
# RPIPLC import
# --------------------------------------------------------
try:
    from librpiplc import rpiplc
    _RPIPLC_IMPORTED = True
except Exception:
    rpiplc = None
    _RPIPLC_IMPORTED = False

# --------------------------------------------------------
# RPIPLC config (ONE init only)
# --------------------------------------------------------
DISABLE_FLOW = os.environ.get("DISABLE_FLOW", "0") == "1"

# Use ONE profile for BOTH DO and AO.
# Default to 38AR because your DO test worked with that.
RPIPLC_BASE = os.environ.get("RPIPLC_BASE", "RPIPLC_V6").strip()
RPIPLC_PROFILE = os.environ.get("RPIPLC_PROFILE", "RPIPLC_38AR").strip()

AO_PIN = os.environ.get("AO_PIN", "A0.5").strip()

MASTER_POWER_PIN = os.environ.get("MASTER_POWER_PIN", "R1.3").strip()
MASTER_POWER_DEFAULT_ON = os.environ.get("MASTER_POWER_DEFAULT_ON", "1") == "1"  # boot HIGH

FLOW_CONTROL_AVAILABLE = False
AO_READY = False
_AO_WARNED = False

MASTER_POWER_AVAILABLE = False
MASTER_POWER_READY = False
MASTER_POWER_STATE = {"enabled": False, "last_change": None, "fault": None}

def rpiplc_init_all():
    """
    Initialise RPIPLC ONCE, then configure:
    - Master power digital output pin
    - Optional AO pin (unless DISABLE_FLOW=1)
    """
    global FLOW_CONTROL_AVAILABLE, AO_READY
    global MASTER_POWER_AVAILABLE, MASTER_POWER_READY

    if rpiplc is None:
        print("[RPIPLC] Not available; AO + master power disabled")
        FLOW_CONTROL_AVAILABLE = False
        AO_READY = False
        MASTER_POWER_AVAILABLE = False
        MASTER_POWER_READY = False
        MASTER_POWER_STATE["fault"] = "rpiplc not available"
        return

    try:
        rpiplc.init(RPIPLC_BASE, RPIPLC_PROFILE)
        print(f"[RPIPLC] init OK: {RPIPLC_BASE} / {RPIPLC_PROFILE}")
    except Exception as e:
        print(f"[RPIPLC] init FAILED: {e}")
        FLOW_CONTROL_AVAILABLE = False
        AO_READY = False
        MASTER_POWER_AVAILABLE = False
        MASTER_POWER_READY = False
        MASTER_POWER_STATE["fault"] = str(e)
        return

    # Master power DO
    try:
        rpiplc.pin_mode(MASTER_POWER_PIN, rpiplc.OUTPUT)
        MASTER_POWER_AVAILABLE = True
        MASTER_POWER_READY = True
        MASTER_POWER_STATE["fault"] = None
        print(f"[PWR] Master power pin ready: {MASTER_POWER_PIN}")

        # Boot default
        rpiplc.digital_write(
            MASTER_POWER_PIN,
            rpiplc.HIGH if MASTER_POWER_DEFAULT_ON else rpiplc.LOW
        )
        MASTER_POWER_STATE["enabled"] = bool(MASTER_POWER_DEFAULT_ON)
        MASTER_POWER_STATE["last_change"] = time.time()
        print(f"[PWR] Boot default set to: {'HIGH/ENABLED' if MASTER_POWER_DEFAULT_ON else 'LOW/DISABLED'}")

    except Exception as e:
        MASTER_POWER_AVAILABLE = False
        MASTER_POWER_READY = False
        MASTER_POWER_STATE["fault"] = str(e)
        print(f"[PWR] Master power setup FAILED: {e}")

    # AO (optional)
    if DISABLE_FLOW:
        print("[AO] DISABLE_FLOW=1 → Flow control forced OFF")
        FLOW_CONTROL_AVAILABLE = False
        AO_READY = False
        return

    try:
        rpiplc.pin_mode(AO_PIN, rpiplc.OUTPUT)
        FLOW_CONTROL_AVAILABLE = True
        AO_READY = True
        print(f"[AO] Ready on {AO_PIN}")
    except Exception as e:
        FLOW_CONTROL_AVAILABLE = False
        AO_READY = False
        print(f"[AO] AO setup FAILED: {e}")

def master_power_set(enable: bool, reason: str = ""):
    if not (MASTER_POWER_AVAILABLE and MASTER_POWER_READY):
        raise RuntimeError("Master power control unavailable")
    rpiplc.digital_write(MASTER_POWER_PIN, rpiplc.HIGH if enable else rpiplc.LOW)
    MASTER_POWER_STATE["enabled"] = bool(enable)
    MASTER_POWER_STATE["last_change"] = time.time()
    MASTER_POWER_STATE["fault"] = None
    EVENTS.publish({"type": "power", "enabled": bool(enable), "reason": reason, "ts": time.time()})
    NOTIFY.publish({"type": "power", "enabled": bool(enable), "reason": reason, "ts": time.time()})
    log("power", f"Master power {'ENABLED' if enable else 'DISABLED'}", reason=reason)

def ao(value: int):
    global _AO_WARNED
    if not (AO_READY and FLOW_CONTROL_AVAILABLE):
        if not _AO_WARNED:
            print("[AO] AO write ignored: Flow control unavailable")
            _AO_WARNED = True
        return
    try:
        rpiplc.analog_write(AO_PIN, value)
    except Exception as e:
        print(f"[AO] write failed: {e}")

FLOW_TO_DAC = {
     5.6: 250,
     8.3: 276,
     9.6: 300,
    20.8: 400,
    32.6: 500,
    39.0: 600,
    54.5: 750,
    76.9: 1000,
   100.0: 1250,
   115.4: 1500,
   117.6: 1750,
   125.0: 2000,
}
FLOW_OPTIONS = sorted(FLOW_TO_DAC.keys())

def dac_for_flow(flow: float) -> int:
    return FLOW_TO_DAC.get(flow, FLOW_TO_DAC[min(FLOW_TO_DAC, key=lambda f: abs(f-flow))])

# --------------------------------------------------------
# Outbound Notifications (webhook + MQTT)
# --------------------------------------------------------
ORCH_WEBHOOK_URL = os.environ.get("ORCH_WEBHOOK_URL", "").strip()
ORCH_WEBHOOK_SECRET = (os.environ.get("ORCH_WEBHOOK_SECRET", "") or "").encode("utf-8")

MQTT_BROKER = os.environ.get("MQTT_BROKER", "").strip()
MQTT_PORT   = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_TOPIC  = os.environ.get("MQTT_TOPIC", "enviro/msf/events").strip()
MQTT_USER   = os.environ.get("MQTT_USER", "")
MQTT_PASS   = os.environ.get("MQTT_PASS", "")

class Notifier:
    def __init__(self):
        self._q = []
        self._qlock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

        self._mqtt_client = None
        self._mqtt_connected = False

    def start(self):
        if MQTT_BROKER and _MQTT_AVAILABLE:
            try:
                self._mqtt_client = mqtt.Client(client_id="msf-api")
                if MQTT_USER:
                    self._mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
                self._mqtt_client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
                self._mqtt_client.loop_start()
                self._mqtt_connected = True
                print(f"[notify] MQTT enabled via broker {MQTT_BROKER}")
            except Exception as e:
                print(f"[notify] MQTT FAILED: {e}")

        self._t.start()

    def stop(self):
        self._stop.set()
        if self._mqtt_client:
            self._mqtt_client.loop_stop()

    def publish(self, event):
        if self._mqtt_client and self._mqtt_connected:
            try:
                self._mqtt_client.publish(MQTT_TOPIC, json.dumps(event))
            except Exception as e:
                print(f"[MQTT ERROR] {e}")

        if ORCH_WEBHOOK_URL:
            with self._qlock:
                self._q.append({"ev": event, "attempt": 0, "next": time.time()})

    def _run(self):
        while not self._stop.is_set():
            now = time.time()
            item = None

            with self._qlock:
                for idx, it in enumerate(self._q):
                    if it["next"] <= now:
                        item = self._q.pop(idx)
                        break

            if not item:
                time.sleep(0.2)
                continue

            ev, attempt = item["ev"], item["attempt"]

            try:
                body = json.dumps(ev).encode("utf8")
                sig = "sha256=" + hmac.new(ORCH_WEBHOOK_SECRET, body, hashlib.sha256).hexdigest()
                if _requests:
                    r = _requests.post(
                        ORCH_WEBHOOK_URL,
                        data=body,
                        headers={"Content-Type": "application/json", "X-Signature": sig},
                        timeout=5
                    )
                    if 200 <= r.status_code < 300:
                        continue
                raise RuntimeError("Webhook failed")
            except Exception:
                backoff = min(60, 2 ** min(attempt, 5))
                item["attempt"] += 1
                item["next"] = time.time() + backoff
                with self._qlock:
                    self._q.append(item)

NOTIFY = Notifier()

# --------------------------------------------------------
# Event Bus (SSE)
# --------------------------------------------------------
class EventBus:
    def __init__(self):
        self._clients = []
        self._lock = threading.Lock()

    def register(self):
        q = []
        with self._lock:
            self._clients.append(q)
        return q

    def unregister(self, q):
        with self._lock:
            if q in self._clients:
                self._clients.remove(q)

    def publish(self, event):
        payload = f"data: {json.dumps(event)}\n\n"
        with self._lock:
            for q in self._clients:
                q.append(payload)

EVENTS = EventBus()

# --------------------------------------------------------
# Logging Helper
# --------------------------------------------------------
def log(kind, msg, **extra):
    ev = {"ts": time.time(), "type": kind, "msg": msg}
    if extra:
        ev.update(extra)
    print(f"[{kind}] {msg}")
    EVENTS.publish(ev)
    NOTIFY.publish(ev)

# --------------------------------------------------------
# App State + Job System
# --------------------------------------------------------
RUN_LOCK = threading.Lock()
RUNNING = {"active": False, "step": "idle", "last": "", "can_stop": False}
STOP_EVENT = threading.Event()

CONFIG = {"filtration_flow": 39.0, "cleaning_flow": 100.0}
JOBS: Dict[str, Dict[str, Any]] = {}
WRITE_RETRIES = 2

def write_cmd(payload, label):
    last_exc = None
    for _ in range(WRITE_RETRIES + 1):
        try:
            commands.write(payload)
            return
        except Exception as e:
            last_exc = e
            time.sleep(0.05)
    raise RuntimeError(f"{label} write failed: {last_exc}")

def emergency_power_cut(reason: str = "emergency"):
    """
    HARD E-STOP:
    - Cut machine power via master relay (LOW)
    - Signal STOP_EVENT so any loops exit
    - Mark RUNNING idle
    """
    STOP_EVENT.set()

    try:
        if MASTER_POWER_AVAILABLE and MASTER_POWER_READY:
            master_power_set(False, reason=reason)
        else:
            log("warn", "Emergency power cut requested but master power control unavailable")
    except Exception as e:
        log("warn", f"Emergency power cut failed: {e}")

    with RUN_LOCK:
        RUNNING.update(active=False, step="idle", last=f"EMERGENCY STOP ({reason})", can_stop=False)

    EVENTS.publish({"type": "emergency", "ts": time.time(), "reason": reason})
    NOTIFY.publish({"type": "emergency", "ts": time.time(), "reason": reason})

    STOP_EVENT.clear()

def run_async(label, fn):
    job_id = uuid.uuid4().hex
    JOBS[job_id] = {"id": job_id, "label": label, "status": "queued", "started": None, "ended": None, "error": None}

    def worker():
        with RUN_LOCK:
            RUNNING.update(active=True, step=label, last="", can_stop=(label == "Sample Filtration"))

        JOBS[job_id].update(status="running", started=time.time())
        EVENTS.publish({"type": "job", "id": job_id, "label": label, "status": "running"})

        try:
            fn()
            msg = f"{label} — done"
            JOBS[job_id].update(status="done", ended=time.time())

            with RUN_LOCK:
                RUNNING.update(active=False, step="idle", last=msg, can_stop=False)

            log("status", msg, job_id=job_id)
            EVENTS.publish({"type": "job", "id": job_id, "label": label, "status": "done"})

        except Exception as e:
            err = str(e)
            JOBS[job_id].update(status="error", error=err, ended=time.time())

            with RUN_LOCK:
                RUNNING.update(active=False, step="idle", last=f"{label} — ERROR: {err}", can_stop=False)

            log("status", f"{label} — ERROR: {err}", job_id=job_id)
            EVENTS.publish({"type": "job", "id": job_id, "label": label, "status": "error", "error": err})

    threading.Thread(target=worker, daemon=True).start()
    return job_id

# --------------------------------------------------------
# Step Implementations
# --------------------------------------------------------
def step_initialisation():
    log("step", "Homing 1")
    write_cmd(command_homing1, "homing1")
    log("step", "Homing 2")
    write_cmd(command_homing2, "homing2")
    time.sleep(90)

def step_filter_loading():
    log("step", "Start roller")
    write_cmd(command_start_roller, "roller_start")
    time.sleep(6)

    log("step", "Stop roller")
    write_cmd(command_stop_roller, "roller_stop")

    log("step", "Base back on")
    write_cmd(command_base_back_on, "base_back_on")
    time.sleep(10)

    log("step", "Base back off")
    write_cmd(command_base_back_off, "base_back_off")

    log("step", "Plate press")
    write_cmd(command_stepper_press, "stepper_press")
    time.sleep(70)

def step_sample_filtration(flow: float):
    if FLOW_CONTROL_AVAILABLE and AO_READY:
        dac = dac_for_flow(flow)
        log("ao", f"Flow {flow} mL/min → DAC {dac}")
        ao(dac)
    else:
        log("ao", f"Flow control disabled → ignoring requested {flow} mL/min")

    log("step", "Open valve 1")
    write_cmd(command_open_valve1, "open_v1")

    log("step", "Open valve 2")
    write_cmd(command_open_valve2, "open_v2")

    log("step", "Open valve 5")
    write_cmd(command_open_valve5, "open_v5")

    log("step", "Start pump")
    write_cmd(command_start_pump, "pump_start")

    with RUN_LOCK:
        RUNNING["can_stop"] = True

    log("wait", "Filtration running... waiting for stop")
    try:
        while not STOP_EVENT.wait(0.2):
            pass
    finally:
        log("step", "Stopping filtration...")
        try:
            write_cmd(command_stop_pump, "pump_stop")
        except Exception:
            pass

        for c in [command_close_valve1, command_close_valve2, command_close_valve5]:
            try:
                time.sleep(0.3)
                write_cmd(c, "close_valve")
            except Exception:
                pass

        with RUN_LOCK:
            RUNNING["can_stop"] = False
        STOP_EVENT.clear()

def step_cleaning(flow: float):
    if FLOW_CONTROL_AVAILABLE and AO_READY:
        dac = dac_for_flow(flow)
        log("ao", f"Cleaning flow {flow} → DAC {dac}")
        ao(dac)
    else:
        log("ao", f"Flow control disabled → ignoring cleaning flow={flow}")

    write_cmd(command_start_pump, "pump_start")
    time.sleep(60)

    write_cmd(command_open_valve3, "open_v3")
    time.sleep(60)

    write_cmd(command_open_valve4, "open_v4")
    time.sleep(60)

    write_cmd(command_stop_pump, "pump_stop")
    time.sleep(60)

    write_cmd(command_close_valve3, "close_v3")
    write_cmd(command_close_valve4, "close_v4")

    write_cmd(command_stepper_open, "stepper_open")
    time.sleep(30)

    write_cmd(command_start_roller, "roller_start")
    time.sleep(2)
    write_cmd(command_stop_roller, "roller_stop")

    write_cmd(command_move_base_front_on, "base_front_on")
    time.sleep(10)
    write_cmd(command_move_base_front_off, "base_front_off")

    write_cmd(command_stepper_press, "stepper_press")
    time.sleep(30)
    write_cmd(command_open_valve2, "open_v2")

    write_cmd(command_start_pump, "pump_start")
    time.sleep(90)
    write_cmd(command_close_valve2, "close_v2")
    time.sleep(60)
    write_cmd(command_stop_pump, "pump_stop")

# --------------------------------------------------------
# Low-Level Safe Actions
# --------------------------------------------------------
def _open_valve(v: int):
    mapping = {1: command_open_valve1, 2: command_open_valve2, 3: command_open_valve3, 4: command_open_valve4, 5: command_open_valve5}
    write_cmd(mapping[v], f"open_valve{v}")

def _close_valve(v: int):
    mapping = {1: command_close_valve1, 2: command_close_valve2, 3: command_close_valve3, 4: command_close_valve4, 5: command_close_valve5}
    write_cmd(mapping[v], f"close_valve{v}")

def _set_flow(flow: float):
    if not (FLOW_CONTROL_AVAILABLE and AO_READY):
        raise RuntimeError("Flow control unavailable on this MSF unit")
    ao(dac_for_flow(flow))

ACTIONS = {
    "open_valve":     lambda a: _open_valve(int(a["id"])),
    "close_valve":    lambda a: _close_valve(int(a["id"])),
    "pump_start":     lambda a: write_cmd(command_start_pump, "pump_start"),
    "pump_stop":      lambda a: write_cmd(command_stop_pump, "pump_stop"),
    "roller_start":   lambda a: write_cmd(command_start_roller, "roller_start"),
    "roller_stop":    lambda a: write_cmd(command_stop_roller, "roller_stop"),
    "base_back_on":   lambda a: write_cmd(command_base_back_on, "base_back_on"),
    "base_back_off":  lambda a: write_cmd(command_base_back_off, "base_back_off"),
    "base_front_on":  lambda a: write_cmd(command_move_base_front_on, "base_front_on"),
    "base_front_off": lambda a: write_cmd(command_move_base_front_off, "base_front_off"),
    "stepper_press":  lambda a: write_cmd(command_stepper_press, "stepper_press"),
    "stepper_open":   lambda a: write_cmd(command_stepper_open, "stepper_open"),
    "set_flow":       lambda a: _set_flow(float(a["flow"])),
}

# --------------------------------------------------------
# API Routes
# --------------------------------------------------------
app = Flask(__name__)
API_KEY = os.environ.get("API_KEY", "").strip()

def require_key(fn):
    @wraps(fn)
    def wrapper(*a, **kw):
        if API_KEY and request.headers.get("X-API-Key") != API_KEY:
            return jsonify({"ok": False, "error": "unauthorized"}), 401
        return fn(*a, **kw)
    return wrapper

@app.get("/healthz")
def healthz():
    return jsonify({
        "ok": True,
        "have_commands": HAVE_COMMANDS,
        "ao_ready": AO_READY,
        "flow_control_available": FLOW_CONTROL_AVAILABLE,
        "flow_disabled_by_env": DISABLE_FLOW,
        "master_power_available": MASTER_POWER_AVAILABLE,
        "master_power_ready": MASTER_POWER_READY,
        "master_power_enabled": MASTER_POWER_STATE["enabled"],
        "master_power_fault": MASTER_POWER_STATE["fault"],
        "master_power_pin": MASTER_POWER_PIN,
        "rpiplc_profile": RPIPLC_PROFILE,
    })

@app.get("/status")
@require_key
def get_status():
    with RUN_LOCK:
        s = dict(RUNNING)
    s["master_power"] = dict(MASTER_POWER_STATE)
    return jsonify(s)

@app.get("/api/events")
@require_key
def sse_events():
    q = EVENTS.register()

    def stream():
        try:
            yield "data: " + json.dumps({"type": "hello", "ts": time.time()}) + "\n\n"
            while True:
                while q:
                    yield q.pop(0)
                time.sleep(0.2)
        finally:
            EVENTS.unregister(q)

    return Response(stream(), mimetype="text/event-stream")

@app.post("/api/power")
@require_key
def api_power():
    data = request.get_json(force=True) or {}
    enable = bool(data.get("enable", False))
    reason = (data.get("reason") or "api_power").strip()
    try:
        master_power_set(enable, reason=reason)
        return jsonify({"ok": True, "enabled": MASTER_POWER_STATE["enabled"]})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

@app.post("/api/emergency_stop")
@require_key
def api_emergency_stop():
    data = request.get_json(force=True) or {}
    reason = (data.get("reason") or "api_emergency_stop").strip()
    emergency_power_cut(reason=reason)
    return jsonify({"ok": True, "message": "Emergency stop executed"})

@app.post("/api/run")
@require_key
def api_run():
    try:
        data = request.get_json(force=True) or {}
    except Exception as e:
        return jsonify({"ok": False, "error": f"Invalid JSON: {e}"}), 400

    step = (data.get("step") or "").strip()
    flow_override = data.get("flow", None)

    with RUN_LOCK:
        if RUNNING["active"]:
            return jsonify({"ok": False, "error": "Already running"}), 409

    if not MASTER_POWER_STATE["enabled"]:
        return jsonify({"ok": False, "error": "Machine power is DISABLED. Enable via /api/power"}), 409

    if step == "initialisation":
        job = run_async("Initialisation", step_initialisation)

    elif step == "filter_loading":
        job = run_async("Filter Loading", step_filter_loading)

    elif step == "sample_filtration":
        if flow_override is not None and not FLOW_CONTROL_AVAILABLE:
            return jsonify({"ok": False, "error": "Flow control unavailable; do NOT provide 'flow'"}), 400
        f = float(flow_override) if flow_override is not None else CONFIG["filtration_flow"]
        job = run_async("Sample Filtration", lambda: step_sample_filtration(f))

    elif step == "cleaning":
        if flow_override is not None and not FLOW_CONTROL_AVAILABLE:
            return jsonify({"ok": False, "error": "Flow control unavailable; do NOT provide 'flow'"}), 400
        f = float(flow_override) if flow_override is not None else CONFIG["cleaning_flow"]
        job = run_async("Cleaning", lambda: step_cleaning(f))

    else:
        return jsonify({"ok": False, "error": "Unknown step"}), 400

    return jsonify({"ok": True, "job_id": job})

@app.post("/api/stop_sample")
@require_key
def api_stop_sample():
    with RUN_LOCK:
        if not RUNNING["active"] or not RUNNING["can_stop"]:
            return jsonify({"ok": False, "error": "Nothing to stop"}), 409
    STOP_EVENT.set()
    EVENTS.publish({"type": "control", "msg": "stop_sample"})
    return jsonify({"ok": True, "message": "Stop signal sent"})

@app.get("/api/jobs/<job_id>")
@require_key
def api_get_job(job_id):
    j = JOBS.get(job_id)
    if not j:
        return jsonify({"ok": False, "error": "not found"}), 404
    return jsonify({"ok": True, "job": j})

@app.post("/api/action")
@require_key
def api_action():
    data = request.get_json(force=True) or {}
    name = (data.get("name") or "").strip()
    args = data.get("args") or {}

    if name not in ACTIONS:
        return jsonify({"ok": False, "error": "unknown action"}), 400

    with RUN_LOCK:
        if RUNNING["active"]:
            return jsonify({"ok": False, "error": "busy"}), 409

    if not MASTER_POWER_STATE["enabled"]:
        return jsonify({"ok": False, "error": "Machine power is DISABLED. Enable via /api/power"}), 409

    try:
        ACTIONS[name](args)
        EVENTS.publish({"type": "action", "name": name, "args": args, "ts": time.time()})
        NOTIFY.publish({"type": "action", "name": name, "args": args, "ts": time.time()})
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

# --------------------------------------------------------
# MAIN
# --------------------------------------------------------
if __name__ == "__main__":
    rpiplc_init_all()

    NOTIFY.start()
    log("hello", "MSF API booted")

    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5000"))
    print(f"[INFO] API key {'ENABLED' if API_KEY else 'DISABLED'}")
    print(f"[INFO] Listening on http://{host}:{port}")

    app.run(host=host, port=port, debug=False, threaded=True)
