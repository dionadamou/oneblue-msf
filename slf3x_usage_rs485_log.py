#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLF3x over SCC1 (RS485) + RPIPLC relay power-cycle recovery
- Logs data to a .txt file (timestamped by default)
- Runs forever until Ctrl+C
- If SCC1 wedges (e.g. SHDLC error 32 / timeouts), it power-cycles SCC1 via RPIPLC relay R1.2
  and automatically reconnects.

USAGE:
  python3 slf3x_scc1_logger_with_relay.py --serial-port /dev/ttySC3

NOTES:
- This script assumes: relay ON (HIGH) = SCC1 powered, relay OFF (LOW) = SCC1 unpowered.
  If your wiring is inverted, swap HIGH/LOW in power_on()/power_off().
"""

import argparse
import datetime
import time
import sys
import traceback

from librpiplc import rpiplc

from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
from sensirion_shdlc_driver.errors import ShdlcDeviceError, ShdlcTimeoutError
from sensirion_uart_scc1.drivers.scc1_slf3x import Scc1Slf3x
from sensirion_uart_scc1.drivers.slf_common import get_flow_unit_label
from sensirion_uart_scc1.scc1_shdlc_device import Scc1ShdlcDevice

# Optional RS485 RTS/DE control via pyserial. Many PLC RS485 ports do NOT need it.
# Keep OFF by default; enable with --force-rs485-rts if you know you need it.
try:
    from serial.rs485 import RS485Settings
except Exception:
    RS485Settings = None


# -------------------------
# RPIPLC RELAY CONTROL
# -------------------------

RPIPLC_BOARD = "RPIPLC_V6"
RPIPLC_MODEL = "RPIPLC_38AR"
RELAY_CH = "R1.2"  # your relay output channel


def relay_init():
    rpiplc.init(RPIPLC_BOARD, RPIPLC_MODEL)
    rpiplc.pin_mode(RELAY_CH, rpiplc.OUTPUT)


def power_on():
    # Assumption: HIGH = power ON
    rpiplc.digital_write(RELAY_CH, rpiplc.HIGH)


def power_off():
    # Assumption: LOW = power OFF
    rpiplc.digital_write(RELAY_CH, rpiplc.LOW)


def power_cycle_scc1(off_ms=200, on_ms=200):
    print("[POWER] SCC1 OFF")
    power_off()
    rpiplc.delay(int(off_ms))

    print("[POWER] SCC1 ON")
    power_on()
    rpiplc.delay(int(on_ms))


# -------------------------
# SERIAL / RS485 HELPERS
# -------------------------

def enable_rs485_mode(shdlc_port):
    """Enable RS485 RTS direction control if supported. OFF by default in main()."""
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


# -------------------------
# LOGGING HELPERS
# -------------------------

def make_log_path(user_path=None, prefix="slf3x_log"):
    if user_path:
        return user_path
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{ts}.txt"


def open_log(path):
    f = open(path, "w", buffering=1)
    f.write("# Timestamp | Flow | Temperature | Flag\n")
    return f


# -------------------------
# SENSOR INIT / RUN
# -------------------------

def init_sensor(port, address, interval_ms):
    """
    Initialize SCC1 + SLF3x and start continuous measurement.
    We do NOT call set_sensor_type() because on your setup that command can wedge.
    """
    device = Scc1ShdlcDevice(ShdlcConnection(port), slave_address=address)
    sensor = Scc1Slf3x(device)

    # Touch a couple of reads (forces early failure if comms are bad)
    sn = sensor.serial_number
    pid = sensor.product_id

    flow_scale, unit = sensor.get_flow_unit_and_scale()
    unit_label = get_flow_unit_label(unit)

    sensor.start_continuous_measurement(interval_ms=interval_ms)

    return sensor, sn, pid, flow_scale, unit_label


def is_wedge_error(exc: Exception) -> bool:
    """
    Detect the 'wedged SCC1' condition. In your logs it appears as error code 32.
    """
    s = str(exc).lower()
    return ("error code 32" in s) or ("returned error 32" in s) or ("shdlc device with address" in s and "error 32" in s)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port", "-p", default="/dev/ttySC3")
    parser.add_argument("--address", "-a", type=int, default=0)
    parser.add_argument("--interval-ms", type=int, default=2)
    parser.add_argument("--log-file", default=None)
    parser.add_argument("--log-prefix", default="slf3x_log")
    parser.add_argument("--rotate-seconds", type=int, default=0,
                        help="If >0, rotate log file every N seconds without restarting sensor.")
    parser.add_argument("--force-rs485-rts", action="store_true",
                        help="Force pyserial RS485 RTS direction control (try OFF first on PLCs).")
    parser.add_argument("--power-cycle-on-start", action="store_true",
                        help="Power cycle SCC1 once at program start for a clean boot.")
    parser.add_argument("--power-off-ms", type=int, default=1500)
    parser.add_argument("--power-on-ms", type=int, default=2500)
    parser.add_argument("--reconnect-delay", type=float, default=1.0,
                        help="Seconds to wait before retry after an error (in addition to power cycle delays).")
    args = parser.parse_args()

    baudrate = 115200

    print("[INFO] Port:", args.serial_port)
    print("[INFO] Relay channel:", RELAY_CH)

    # Init RPIPLC relay
    relay_init()
    # Ensure SCC1 is powered on at least
    power_on()
    rpiplc.delay(200)

    if args.power_cycle_on_start:
        power_cycle_scc1(off_ms=args.power_off_ms, on_ms=args.power_on_ms)

    # Open initial log
    log_path = make_log_path(args.log_file, prefix=args.log_prefix)
    log_file = open_log(log_path)
    print("[INFO] Logging to:", log_path)

    next_rotate = time.time() + args.rotate_seconds if args.rotate_seconds and args.rotate_seconds > 0 else None

    sensor = None

    try:
        while True:
            port = None
            sensor = None

            try:
                # Open port
                port = ShdlcSerialPort(port=args.serial_port, baudrate=baudrate)

                if args.force_rs485_rts:
                    enable_rs485_mode(port)
                    print("[INFO] RS485 RTS mode: ON")
                else:
                    print("[INFO] RS485 RTS mode: OFF")

                # Init sensor
                sensor, sn, pid, flow_scale, unit_label = init_sensor(
                    port=port,
                    address=args.address,
                    interval_ms=args.interval_ms
                )

                print("[OK] Connected.")
                print("serial_number:", sn)
                print("product id:", pid)
                print("Flow; Temperature; Flag")

                # Stream loop
                while True:
                    if next_rotate is not None and time.time() >= next_rotate:
                        # Rotate log file without restarting SCC1
                        try:
                            log_file.write("# Rotating log\n")
                            log_file.close()
                        except Exception:
                            pass
                        log_path = make_log_path(None, prefix=args.log_prefix)
                        log_file = open_log(log_path)
                        print("[INFO] Logging to:", log_path)
                        next_rotate = time.time() + args.rotate_seconds

                    remaining, lost, data = sensor.read_extended_buffer()

                    if lost > 0:
                        msg = f"# WARN lost={lost}\n"
                        print("[WARN]", lost, "samples lost")
                        log_file.write(msg)

                    for flow, temperature, flag in data:
                        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                        flow_val = flow / flow_scale
                        temp_val = temperature / 200.0

                        line = (
                            f"{timestamp} | "
                            f"flow={flow_val:.6f} {unit_label} | "
                            f"temp={temp_val:.2f} C | "
                            f"flag={flag}\n"
                        )
                        print(line.strip())
                        log_file.write(line)

            except KeyboardInterrupt:
                print("\n[INFO] Ctrl+C received, stopping...")
                break

            except (ShdlcDeviceError, ShdlcTimeoutError) as e:
                print("[WARN] SHDLC communication error:", e)
                traceback.print_exc(file=sys.stdout)

                # Power-cycle SCC1 on wedge-like errors OR on repeated timeouts
                print("[INFO] Power cycling SCC1 to recover...")
                try:
                    power_cycle_scc1(off_ms=args.power_off_ms, on_ms=args.power_on_ms)
                except Exception as pe:
                    print("[ERROR] Relay power-cycle failed:", pe)

                time.sleep(args.reconnect_delay)

            except Exception as e:
                print("[ERROR] Unexpected exception:", e)
                traceback.print_exc(file=sys.stdout)

                # If it smells like the wedge, power-cycle too
                if is_wedge_error(e):
                    print("[INFO] Looks like SCC1 wedge. Power cycling...")
                    try:
                        power_cycle_scc1(off_ms=args.power_off_ms, on_ms=args.power_on_ms)
                    except Exception as pe:
                        print("[ERROR] Relay power-cycle failed:", pe)
                    time.sleep(args.reconnect_delay)
                else:
                    # For other exceptions, small delay then retry
                    time.sleep(args.reconnect_delay)

            finally:
                # Best-effort stop and close
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

    finally:
        try:
            log_file.write("# Measurement stopped\n")
            log_file.close()
        except Exception:
            pass

        # Leave SCC1 powered ON at exit (usually desired)
        try:
            power_on()
        except Exception:
            pass

        print("[OK] Exited cleanly.")


if __name__ == "__main__":
    main()
