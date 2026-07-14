#!/usr/bin/env python3
"""Live raw-telemetry console for the boat link.

Opens the shore serial port directly, continuously prompts the boat by sending
command frames (the boat only transmits telemetry in reply to a command), and
prints every raw line received, exactly as it arrives over the wire.

Usage:
    python raw_monitor.py [PORT] [BAUD]
    python raw_monitor.py COM4 115200

Notes:
    * Only ONE process may hold the serial port. Disconnect the control server
      from the port first (POST /api/disconnect) or stop it.
    * The command frame format matches control_server.build_serial_line():
      "<left>,<right>,<winch>,<radarAngle>\\n".
    * Press Ctrl+C to quit.
"""
from __future__ import annotations

import sys
import time

import serial  # pyserial


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM4"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as exc:  # noqa: BLE001 - surface any open error to the user
        print(f"[FATAL] could not open {port} @ {baud}: {exc}")
        print("        Is the control server still holding the port? "
              "Disconnect it first (POST /api/disconnect).")
        return 1

    print(f"[open] {port} @ {baud}. Sending prompts + dumping RAW lines. Ctrl+C to quit.")
    print("       (allowing 2s for the shore board to reset after port open...)")
    time.sleep(2.0)

    buf = b""
    last_cmd = 0.0
    last_rx = time.time()
    started = time.time()
    line_count = 0
    angle_seq = [0, 15, 30, 45, 60, 75, 60, 45, 30, 15]
    seq_i = 0

    try:
        while True:
            now = time.time()

            # Prompt the boat every 250ms with a stop command + sweeping angle.
            if now - last_cmd >= 0.25:
                angle = angle_seq[seq_i % len(angle_seq)]
                seq_i += 1
                try:
                    ser.write(f"0,0,0,{angle}\n".encode())
                except Exception as exc:  # noqa: BLE001
                    print(f"[write-error] {exc}")
                last_cmd = now

            data = ser.read(256)
            if data:
                last_rx = now
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line_count += 1
                    ts = time.strftime("%H:%M:%S", time.localtime(now))
                    text = raw.decode(errors="replace").rstrip("\r")
                    parts = text.split(",")
                    tag = ""
                    if len(parts) == 5:
                        try:
                            v = [int(p) for p in parts]
                            if v[0] == v[1] == v[2] == v[3] == 111:
                                tag = "   <diag/111 ack>"
                            else:
                                tag = (f"   radar={v[0]} front={v[1]} "
                                       f"left={v[2]} right={v[3]} angle={v[4]}")
                        except ValueError:
                            tag = "   <non-numeric>"
                    print(f"[{ts}] #{line_count:<5} RAW: {text!r}{tag}")
            else:
                # Heartbeat so the user sees the console is alive but silent.
                if now - last_rx >= 3.0:
                    silence = now - last_rx
                    print(f"[{time.strftime('%H:%M:%S')}] ... no data for "
                          f"{silence:4.1f}s (boat silent / out of range / off)")
                    last_rx = now  # throttle the message to once per 3s
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        dur = time.time() - started
        print(f"\n[close] {port}. {line_count} raw lines in {dur:.0f}s.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
