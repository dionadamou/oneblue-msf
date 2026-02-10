#!/usr/bin/env python3
"""
control_api.py — API-only controller for MSF device (OneBlue MSF)

Includes:
- High-level steps + low-level actions
- SSE events + optional webhook/MQTT notifications
- RPIPLC AO (optional; disabled if DISABLE_FLOW=1 or missing)
- Master power relay enable on RPIPLC DO (R1.3 by default)
    - HIGH  = machine powered
    - LOW   = machine power cut (E-STOP)
- Boot default: master power HIGH (configurable)

Homing (timeout-based signature detection):
- Uses RPIPLC digital input HOME_LIMIT_PIN (default I1.5)
- Detects HIGH -> LOW -> HIGH
- Timeout HOME_TIMEOUT_S (default 90s)
- Poll interval HOME_POLL_MS (default 10ms)
- Stable debounce HOME_STABLE_COUNT (default 5)

Sample filtration by TARGET VOLUME (mL):
- Uses SLF3x over SCC1 to integrate volume (mL) from flow (mL/min)
- /api/run step="sample_filtration" requires "volume_ml"
- /status includes "flowmeter" data (ok, flow, total_ml, err, etc.)

NEW: SCC1 power-cycle recovery (like your logger):
- Controls flowmeter SCC1 power via RPIPLC relay FLOW_PWR_RELAY_PIN (default R1.2)
- Optionally power-cycles SCC1 at boot (FLOW_POWER_CYCLE_ON_BOOT=1 default)
- On SHDLC timeouts/errors inside flow thread: power-cycle SCC1 and reconnect
- Adds /api/action name="flow_power_cycle" (manual from other Pi)

ENV VARS (key ones):
  API_KEY=...
  PORT=5001

  # RPIPLC
  RPIPLC_BASE=RPIPLC_V6
  RPIPLC_PROFILE=RPIPLC_38AR
  MASTER_POWER_PIN=R1.3
  MASTER_POWER_DEFAULT_ON=1
  AO_PIN=A0.5
  DISABLE_FLOW=0

  # Homing input
  HOME_LIMIT_PIN=I1.5
  HOME_TIMEOUT_S=90
  HOME_POLL_MS=10
  HOME_STABLE_COUNT=5

  # Flowmeter
  ENABLE_FLOWMETER=1
  FLOW_PORT=/dev/ttySC3
  FLOW_ADDR=0
  FLOW_INTERVAL_MS=2
  FLOW_CAL=1.0

  # Flow glitch filtering
  MAX_FLOW_ML_MIN=90.0
  ABS_OUTLIER_ML_MIN=25.0
  REL_OUTLIER_FACTOR=2.5
  MIN_GOOD_SAMPLES=3
  MAX_STEP_JUMP_ML_MIN=40.0
  CAP_DV_PER_BATCH_ML=5.0
  COMPENSATE_LOST_SAMPLES=1

  # SCC1 power relay
  FLOW_PWR_RELAY_PIN=R1.2
  FLOW_PWR_ASSUME_HIGH_ON=1
  FLOW_POWER_CYCLE_ON_BOOT=1
  FLOW_POWER_OFF_MS=1500
  FLOW_POWER_ON_MS=2500
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
# Flowmeter (SLF3x over SCC1 / SHDLC) imports (optional)
# --------------------------------------------------------
try:
    from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
    from sensirion_shdlc_driver.errors import ShdlcDeviceError, ShdlcTimeoutError
    from sensirion_uart_scc1.drivers.scc1_slf3x import Scc1Slf3x
    from sensirion_uart_scc1.drivers.slf_common import get_flow_unit_label
    from sensirion_uart_scc1.scc1_shdlc_device import Scc1ShdlcDevice
    FLOW_LIBS_OK = True
    FLOW_LIBS_ERR = ""
except Exception as e:
    FLOW_LIBS_OK = False
    FLOW_LIBS_ERR = str(e)

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

RPIPLC_BASE = os.environ.get("RPIPLC_BASE", "RPIPLC_V6").strip()
RPIPLC_PROFILE = os.environ.get("RPIPLC_PROFILE", "RPIPLC_38AR").strip()

AO_PIN = os.environ.get("AO_PIN", "A0.5").strip()

MASTER_POWER_PIN = os.environ.get("MASTER_POWER_PIN", "R1.3").strip()
MASTER_POWER_DEFAULT_ON = os.environ.get("MASTER_POWER_DEFAULT_ON", "1") == "1"  # boot HIGH

# ----- homing input -----
HOME_LIMIT_PIN = os.environ.get("HOME_LIMIT_PIN", "I1.5").strip()
HOME_TIMEOUT_S = float(os.environ.get("HOME_TIMEOUT_S", "90"))
HOME_POLL_MS = int(os.environ.get("HOME_POLL_MS", "10"))
HOME_STABLE_COUNT = int(os.environ.get("HOME_STABLE_COUNT", "5"))

FLOW_CONTROL_AVAILABLE = False
AO_READY = False
_AO_WARNED = False

MASTER_POWER_AVAILABLE = False
MASTER_POWER_READY = False
MASTER_POWER_STATE = {"enabled": False, "last_change": None, "fault": None}

# --------------------------------------------------------
# Flowmeter + volume integration
# --------------------------------------------------------
ENABLE_FLOWMETER = os.environ.get("ENABLE_FLOWMETER", "1") == "1"
FLOW_PORT = os.environ.get("FLOW_PORT", "/dev/ttySC3").strip()
FLOW_ADDR = int(os.environ.get("FLOW_ADDR", "0"))
FLOW_INTERVAL_MS = int(os.environ.get("FLOW_INTERVAL_MS", "2"))

FLOW_CAL = float(os.environ.get("FLOW_CAL", "1.0"))

MAX_FLOW_ML_MIN = float(os.environ.get("MAX_FLOW_ML_MIN", "90.0"))
ABS_OUTLIER_ML_MIN = float(os.environ.get("ABS_OUTLIER_ML_MIN", "25.0"))
REL_OUTLIER_FACTOR = float(os.environ.get("REL_OUTLIER_FACTOR", "2.5"))
MIN_GOOD_SAMPLES = int(os.environ.get("MIN_GOOD_SAMPLES", "3"))
MAX_STEP_JUMP_ML_MIN = float(os.environ.get("MAX_STEP_JUMP_ML_MIN", "40.0"))
CAP_DV_PER_BATCH_ML = float(os.environ.get("CAP_DV_PER_BATCH_ML", "5.0"))
COMPENSATE_LOST_SAMPLES = os.environ.get("COMPENSATE_LOST_SAMPLES", "1") == "1"

FLOW_LOCK = threading.Lock()
FLOW_STATE = {
    "ok": False,
    "ml_min": 0.0,
    "temp_c": 0.0,
    "unit": "mL/min",
    "last_ts": 0.0,
    "err": "",
    "total_ml": 0.0,  # integrated since API boot
}

# --------------------------------------------------------
# SCC1 power relay (R1.2) for flowmeter recovery
# --------------------------------------------------------
FLOW_PWR_RELAY_PIN = os.environ.get("FLOW_PWR_RELAY_PIN", "R1.2").strip()
FLOW_PWR_ASSUME_HIGH_ON = os.environ.get("FLOW_PWR_ASSUME_HIGH_ON", "1") == "1"  # HIGH=ON

FLOW_POWER_CYCLE_ON_BOOT = os.environ.get("FLOW_POWER_CYCLE_ON_BOOT", "1") == "1"
FLOW_POWER_OFF_MS = int(os.environ.get("FLOW_POWER_OFF_MS", "1500"))
FLOW_POWER_ON_MS  = int(os.environ.get("FLOW_POWER_ON_MS", "2500"))

def flow_power_on():
    if rpiplc is None:
        raise RuntimeError("rpiplc not available")
    rpiplc.digital_write(FLOW_PWR_RELAY_PIN, rpiplc.HIGH if FLOW_PWR_ASSUME_HIGH_ON else rpiplc.LOW)

def flow_power_off():
    if rpiplc is None:
        raise RuntimeError("rpiplc not available")
    rpiplc.digital_write(FLOW_PWR_RELAY_PIN, rpiplc.LOW if FLOW_PWR_ASSUME_HIGH_ON else rpiplc.HIGH)

def flow_power_cycle(off_ms=None, on_ms=None):
    off_ms = FLOW_POWER_OFF_MS if off_ms is None else int(off_ms)
    on_ms  = FLOW_POWER_ON_MS  if on_ms  is None else int(on_ms)
    log("flow_power", f"SCC1 power cycle: OFF {off_ms}ms, ON {on_ms}ms")
    try:
        flow_power_off()
        rpiplc.delay(off_ms)
        flow_power_on()
        rpiplc.delay(on_ms)
    except Exception as e:
        log("flow_power", f"Power cycle failed: {e}")
        raise

def rpiplc_init_all():
    """
    Initialise RPIPLC ONCE, then configure:
    - Master power digital output pin
    - Optional AO pin (unless DISABLE_FLOW=1)
    - Home limit input pin (HOME_LIMIT_PIN)
    - SCC1 power relay pin (FLOW_PWR_RELAY_PIN)
    """
    global FLOW_CONTROL_AVAILABLE, AO_READY
    global MASTER_POWER_AVAILABLE, MASTER_POWER_READY

    if rpiplc is None:
        print("[RPIPLC] Not available; AO + master power + home input disabled")
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

    # Home limit input
    try:
        rpiplc.pin_mode(HOME_LIMIT_PIN, rpiplc.INPUT)
        print(f"[HOME] Limit input ready: {HOME_LIMIT_PIN}")
    except Exception as e:
        print(f"[HOME] Limit input setup FAILED on {HOME_LIMIT_PIN}: {e}")

    # SCC1 power relay pin
    try:
        rpiplc.pin_mode(FLOW_PWR_RELAY_PIN, rpiplc.OUTPUT)
        print(f"[FLOW_PWR] Relay pin ready: {FLOW_PWR_RELAY_PIN}")
        try:
            flow_power_on()
            rpiplc.delay(200)
        except Exception:
            pass

        if FLOW_POWER_CYCLE_ON_BOOT:
            try:
                # clean SCC1 boot (your setup needs it)
                flow_power_cycle(FLOW_POWER_OFF_MS, FLOW_POWER_ON_MS)
            except Exception:
                pass

    except Exception as e:
        print(f"[FLOW_PWR] Relay setup FAILED on {FLOW_PWR_RELAY_PIN}: {e}")

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
# Master power + AO
# --------------------------------------------------------
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
        rpiplc.analog_write(AO_PIN, int(value))
    except Exception as e:
        print(f"[AO] write failed: {e}")

# --------------------------------------------------------
# Flow->DAC table (your updated one)
# --------------------------------------------------------
FLOW_TO_DAC = {
     5: 250,
     9: 300,
     17: 400,
     25: 500,
     34: 600,
     43: 700,
     51: 800,
     59: 900,
}
FLOW_OPTIONS = sorted(FLOW_TO_DAC.keys())

def dac_for_flow(flow: float) -> int:
    f = float(flow)
    return int(FLOW_TO_DAC.get(f, FLOW_TO_DAC[min(FLOW_TO_DAC, key=lambda x: abs(x - f))]))

# --------------------------------------------------------
# Flow thread (SLF3x) — integration to total_ml
# --------------------------------------------------------
def _median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    return s[mid] if (n % 2 == 1) else 0.5 * (s[mid - 1] + s[mid])

def _init_slf3x_sensor():
    port = ShdlcSerialPort(port=FLOW_PORT, baudrate=115200)
    device = Scc1ShdlcDevice(ShdlcConnection(port), slave_address=FLOW_ADDR)
    sensor = Scc1Slf3x(device)

    flow_scale, unit = sensor.get_flow_unit_and_scale()
    unit_label = get_flow_unit_label(unit)

    sensor.start_continuous_measurement(interval_ms=FLOW_INTERVAL_MS)
    return port, sensor, flow_scale, unit_label

def flow_reader_thread():
    last_good_flow = 0.0
    last_good_temp = 0.0
    last_batch_t = time.monotonic()

    while True:
        port = None
        sensor = None
        try:
            if not ENABLE_FLOWMETER:
                with FLOW_LOCK:
                    FLOW_STATE["ok"] = False
                    FLOW_STATE["err"] = "ENABLE_FLOWMETER=0"
                    FLOW_STATE["last_ts"] = time.time()
                time.sleep(2.0)
                continue

            if not FLOW_LIBS_OK:
                with FLOW_LOCK:
                    FLOW_STATE["ok"] = False
                    FLOW_STATE["err"] = f"flow libs missing: {FLOW_LIBS_ERR}"
                    FLOW_STATE["last_ts"] = time.time()
                time.sleep(2.0)
                continue

            # Ensure SCC1 is ON (best-effort)
            try:
                if rpiplc is not None:
                    flow_power_on()
                    rpiplc.delay(100)
            except Exception:
                pass

            port, sensor, flow_scale, unit_label = _init_slf3x_sensor()

            with FLOW_LOCK:
                FLOW_STATE["ok"] = True
                FLOW_STATE["unit"] = unit_label
                FLOW_STATE["err"] = ""
                FLOW_STATE["last_ts"] = time.time()

            last_batch_t = time.monotonic()

            while True:
                remaining, lost, data = sensor.read_extended_buffer()
                if not data:
                    continue

                now_t = time.monotonic()
                dt_total = now_t - last_batch_t
                last_batch_t = now_t

                raw_flow = [(FLOW_CAL * (f / flow_scale)) for (f, temp, flag) in data]
                raw_temp = [(temp / 200.0) for (f, temp, flag) in data]

                # clamp
                clamped = []
                for x in raw_flow:
                    if x < 0:
                        x = 0.0
                    if x > MAX_FLOW_ML_MIN:
                        x = MAX_FLOW_ML_MIN
                    clamped.append(x)

                med = _median(clamped)

                # outlier rejection vs median
                good = []
                for x in clamped:
                    if abs(x - med) <= ABS_OUTLIER_ML_MIN and x <= (med * REL_OUTLIER_FACTOR + 1e-9):
                        good.append(x)

                if len(good) < MIN_GOOD_SAMPLES:
                    with FLOW_LOCK:
                        FLOW_STATE["ok"] = True  # comms ok, data glitch
                        FLOW_STATE["ml_min"] = float(last_good_flow)
                        FLOW_STATE["temp_c"] = float(last_good_temp)
                        FLOW_STATE["last_ts"] = time.time()
                        FLOW_STATE["err"] = "glitch batch ignored"
                    continue

                flow_mean = sum(good) / len(good)
                temp_mean = (sum(raw_temp) / len(raw_temp)) if raw_temp else last_good_temp

                # jump rejection
                if abs(flow_mean - last_good_flow) > MAX_STEP_JUMP_ML_MIN:
                    with FLOW_LOCK:
                        FLOW_STATE["ok"] = True
                        FLOW_STATE["ml_min"] = float(last_good_flow)
                        FLOW_STATE["temp_c"] = float(last_good_temp)
                        FLOW_STATE["last_ts"] = time.time()
                        FLOW_STATE["err"] = "flow jump ignored"
                    continue

                lost_n = int(lost) if (lost is not None and lost > 0) else 0
                n_total = len(good) + (lost_n if COMPENSATE_LOST_SAMPLES else 0)
                if n_total <= 0:
                    continue

                dt_per = dt_total / float(n_total)
                dV_ml = (sum(good) * dt_per) / 60.0

                if COMPENSATE_LOST_SAMPLES and lost_n > 0:
                    dV_ml += (lost_n * flow_mean * dt_per) / 60.0

                # safety cap per batch
                if dV_ml < 0:
                    dV_ml = 0.0
                if dV_ml > CAP_DV_PER_BATCH_ML:
                    dV_ml = CAP_DV_PER_BATCH_ML

                last_good_flow = flow_mean
                last_good_temp = temp_mean

                with FLOW_LOCK:
                    FLOW_STATE["ok"] = True
                    FLOW_STATE["ml_min"] = float(flow_mean)
                    FLOW_STATE["temp_c"] = float(temp_mean)
                    FLOW_STATE["last_ts"] = time.time()
                    FLOW_STATE["err"] = ""
                    FLOW_STATE["total_ml"] = float(FLOW_STATE.get("total_ml", 0.0) + dV_ml)

        except (ShdlcDeviceError, ShdlcTimeoutError) as e:
            with FLOW_LOCK:
                FLOW_STATE["ok"] = False
                FLOW_STATE["err"] = f"SHDLC error: {e}"
                FLOW_STATE["last_ts"] = time.time()
            log("flow", f"SHDLC error -> power cycle SCC1: {e}")
            try:
                if rpiplc is not None:
                    flow_power_cycle()
            except Exception:
                pass
            time.sleep(1.0)

        except Exception as e:
            with FLOW_LOCK:
                FLOW_STATE["ok"] = False
                FLOW_STATE["err"] = f"Flow thread error: {e}"
                FLOW_STATE["last_ts"] = time.time()
            log("flow", f"Flow thread error -> power cycle SCC1: {e}")
            try:
                if rpiplc is not None:
                    flow_power_cycle()
            except Exception:
                pass
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

def get_total_ml() -> float:
    with FLOW_LOCK:
        return float(FLOW_STATE.get("total_ml", 0.0) or 0.0)

# --------------------------------------------------------
# HOMING WAIT (HIGH -> LOW -> HIGH) with timeout
# --------------------------------------------------------
def _read_home_pin() -> int:
    return 1 if rpiplc.digital_read(HOME_LIMIT_PIN) else 0

def _wait_stable(target: int, stable_count: int, poll_s: float,
                 deadline: float, stop_event: threading.Event = None) -> None:
    consec = 0
    while True:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError("Aborted (STOP_EVENT set)")
        if time.monotonic() > deadline:
            raise TimeoutError(f"Timeout waiting for {HOME_LIMIT_PIN} stable={target}")

        v = _read_home_pin()
        if v == target:
            consec += 1
            if consec >= stable_count:
                return
        else:
            consec = 0
        time.sleep(poll_s)

def wait_for_homing_signature(stop_event: threading.Event = None) -> None:
    """
    Detect: HIGH -> LOW -> HIGH on HOME_LIMIT_PIN.
    pressed=LOW, released=HIGH
    """
    if rpiplc is None:
        raise RuntimeError("rpiplc not available (cannot read home pin)")
    poll_s = max(0.001, HOME_POLL_MS / 1000.0)
    deadline = time.monotonic() + HOME_TIMEOUT_S

    log("home", f"Watching {HOME_LIMIT_PIN} for HIGH->LOW->HIGH (timeout={HOME_TIMEOUT_S}s poll={HOME_POLL_MS}ms stable={HOME_STABLE_COUNT})")

    log("home", "Stage 0: waiting for HIGH (released)")
    _wait_stable(1, HOME_STABLE_COUNT, poll_s, deadline, stop_event)

    log("home", "Stage 1: waiting for LOW (pressed)")
    _wait_stable(0, HOME_STABLE_COUNT, poll_s, deadline, stop_event)

    log("home", "Stage 2: waiting for HIGH again (released)")
    _wait_stable(1, HOME_STABLE_COUNT, poll_s, deadline, stop_event)

    log("home", "Signature detected → HOMED")

# --------------------------------------------------------
# App State + Job System
# --------------------------------------------------------
RUN_LOCK = threading.Lock()
RUNNING = {"active": False, "step": "idle", "last": "", "can_stop": False}
STOP_EVENT = threading.Event()

CONFIG = {"filtration_flow": 17.0, "cleaning_flow": 59.0}
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
    wait_for_homing_signature(stop_event=STOP_EVENT)

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
    time.sleep(30)

def step_sample_filtration(flow: float, target_ml: float):
    if target_ml is None:
        raise ValueError("target_ml is required")
    target_ml = float(target_ml)
    if target_ml <= 0:
        raise ValueError("target_ml must be > 0")

    # Require flowmeter OK for volume-based stop
    with FLOW_LOCK:
        fm_ok = bool(FLOW_STATE.get("ok", False))
        fm_err = str(FLOW_STATE.get("err", ""))
    if not (ENABLE_FLOWMETER and fm_ok):
        raise RuntimeError(f"Flowmeter not OK (ENABLE_FLOWMETER={ENABLE_FLOWMETER}) err={fm_err}")

    baseline = get_total_ml()
    log("wait", f"Volume baseline set: total_ml={baseline:.3f} mL")

    # AO set (optional)
    if FLOW_CONTROL_AVAILABLE and AO_READY:
        dac = dac_for_flow(flow)
        log("ao", f"Flow {flow} mL/min → DAC {dac}")
        ao(dac)
    else:
        log("ao", f"Flow control disabled → ignoring requested {flow} mL/min")

    # valves + pump
    log("step", "Open valve 1"); write_cmd(command_open_valve1, "open_v1")
    log("step", "Open valve 2"); write_cmd(command_open_valve2, "open_v2")
    log("step", "Open valve 5"); write_cmd(command_open_valve5, "open_v5")
    log("step", "Start pump");   write_cmd(command_start_pump, "pump_start")

    with RUN_LOCK:
        RUNNING["can_stop"] = True

    log("wait", f"Filtration running... target={target_ml:.2f} mL")
    try:
        while not STOP_EVENT.wait(0.2):
            delivered = max(0.0, get_total_ml() - baseline)
            if delivered >= target_ml:
                log("wait", f"Target reached ({delivered:.2f} mL) → stopping")
                break
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
    time.sleep(75)

    write_cmd(command_open_valve3, "open_v3")
    time.sleep(75)

    write_cmd(command_open_valve4, "open_v4")
    time.sleep(75)

    write_cmd(command_stop_pump, "pump_stop")
    time.sleep(75)

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
    time.sleep(75)
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

def _set_dac(dac: int):
    if not (FLOW_CONTROL_AVAILABLE and AO_READY):
        raise RuntimeError("Flow control unavailable on this MSF unit")
    d = int(dac)
    d = max(0, min(4095, d))
    ao(d)

def _flow_power_cycle(args: dict):
    off_ms = int(args.get("off_ms", FLOW_POWER_OFF_MS))
    on_ms  = int(args.get("on_ms", FLOW_POWER_ON_MS))
    flow_power_cycle(off_ms=off_ms, on_ms=on_ms)

ACTIONS = {
    "open_valve":       lambda a: _open_valve(int(a["id"])),
    "close_valve":      lambda a: _close_valve(int(a["id"])),
    "pump_start":       lambda a: write_cmd(command_start_pump, "pump_start"),
    "pump_stop":        lambda a: write_cmd(command_stop_pump, "pump_stop"),
    "roller_start":     lambda a: write_cmd(command_start_roller, "roller_start"),
    "roller_stop":      lambda a: write_cmd(command_stop_roller, "roller_stop"),
    "base_back_on":     lambda a: write_cmd(command_base_back_on, "base_back_on"),
    "base_back_off":    lambda a: write_cmd(command_base_back_off, "base_back_off"),
    "base_front_on":    lambda a: write_cmd(command_move_base_front_on, "base_front_on"),
    "base_front_off":   lambda a: write_cmd(command_move_base_front_off, "base_front_off"),
    "stepper_press":    lambda a: write_cmd(command_stepper_press, "stepper_press"),
    "stepper_open":     lambda a: write_cmd(command_stepper_open, "stepper_open"),
    "set_flow":         lambda a: _set_flow(float(a["flow"])),
    "set_dac":          lambda a: _set_dac(int(a["dac"])),
    "flow_power_cycle": lambda a: _flow_power_cycle(a),
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

        "home_limit_pin": HOME_LIMIT_PIN,
        "home_timeout_s": HOME_TIMEOUT_S,
        "home_poll_ms": HOME_POLL_MS,
        "home_stable_count": HOME_STABLE_COUNT,

        "enable_flowmeter": ENABLE_FLOWMETER,
        "flow_libs_ok": FLOW_LIBS_OK,
        "flow_libs_err": FLOW_LIBS_ERR,
        "flow_port": FLOW_PORT,
        "flow_addr": FLOW_ADDR,
        "flow_interval_ms": FLOW_INTERVAL_MS,

        "flow_pwr_relay_pin": FLOW_PWR_RELAY_PIN,
        "flow_power_cycle_on_boot": FLOW_POWER_CYCLE_ON_BOOT,
        "flow_power_off_ms": FLOW_POWER_OFF_MS,
        "flow_power_on_ms": FLOW_POWER_ON_MS,
    })

@app.get("/status")
@require_key
def get_status():
    with RUN_LOCK:
        s = dict(RUNNING)
    s["master_power"] = dict(MASTER_POWER_STATE)
    with FLOW_LOCK:
        s["flowmeter"] = dict(FLOW_STATE)
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
        STOP_EVENT.clear()
        job = run_async("Initialisation", step_initialisation)

    elif step == "filter_loading":
        job = run_async("Filter Loading", step_filter_loading)

    elif step == "sample_filtration":
        STOP_EVENT.clear()

        vol = data.get("volume_ml", None)
        if vol is None:
            return jsonify({"ok": False, "error": "sample_filtration requires 'volume_ml'"}), 400

        if flow_override is not None and not FLOW_CONTROL_AVAILABLE:
            return jsonify({"ok": False, "error": "Flow control unavailable; do NOT provide 'flow'"}), 400

        f = float(flow_override) if flow_override is not None else CONFIG["filtration_flow"]
        v = float(vol)
        job = run_async("Sample Filtration", lambda: step_sample_filtration(f, v))

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

    # Start flowmeter thread (if enabled)
    if ENABLE_FLOWMETER:
        threading.Thread(target=flow_reader_thread, daemon=True).start()
        log("flow", f"Flowmeter thread started on {FLOW_PORT}")
    else:
        log("flow", "Flowmeter disabled (ENABLE_FLOWMETER=0)")

    NOTIFY.start()
    log("hello", "MSF API booted")

    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "5001"))
    print(f"[INFO] API key {'ENABLED' if API_KEY else 'DISABLED'}")
    print(f"[INFO] Listening on http://{host}:{port}")

    app.run(host=host, port=port, debug=False, threaded=True)
