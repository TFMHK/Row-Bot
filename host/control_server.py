from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import threading
import time
from dataclasses import dataclass, field, asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import serial
from serial.tools import list_ports


ROOT_DIR = Path(__file__).resolve().parent
WEB_DIR = ROOT_DIR / "web"


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


@dataclass
class Telemetry:
    usRadar: int | None = None
    usFront: int | None = None
    usLeft: int | None = None
    usRight: int | None = None
    radarAngle: int = 90


@dataclass
class CommandState:
    leftSpeed: int = 0
    rightSpeed: int = 0
    winchSpeed: int = 0
    radarAngle: int = 90


@dataclass
class BridgeState:
    connected: bool = False
    mockEnabled: bool = False
    serialPort: str = ""
    baudRate: int = 115200
    lastError: str = ""
    lastMessageAt: float | None = None
    telemetry: Telemetry = field(default_factory=Telemetry)
    command: CommandState = field(default_factory=CommandState)


class SerialBridge:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self._mock_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._state = BridgeState()
        self._mock_thread.start()

    def list_ports(self) -> list[str]:
        return [port.device for port in list_ports.comports()]

    def connect(self, port_name: str, baud_rate: int = 115200) -> BridgeState:
        with self._lock:
            self._disconnect_locked(send_stop=False)
            self._state.mockEnabled = False
            self._serial = serial.Serial(port_name, baud_rate, timeout=0.2)
            time.sleep(2.0)
            self._state.connected = True
            self._state.serialPort = port_name
            self._state.baudRate = baud_rate
            self._state.lastError = ""
            self._stop_reader.clear()
            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()
            return self.snapshot()

    def disconnect(self) -> BridgeState:
        with self._lock:
            self._disconnect_locked(send_stop=True)
            self._state.mockEnabled = False
            return self.snapshot()

    def set_mock_enabled(self, enabled: bool) -> BridgeState:
        with self._lock:
            if enabled:
                self._disconnect_locked(send_stop=True)
                self._state.mockEnabled = True
                self._state.serialPort = "MOCK"
                self._state.lastError = ""
                self._state.lastMessageAt = time.time()
                self._state.telemetry = Telemetry(usRadar=180, usFront=120, usLeft=95, usRight=105, radarAngle=90)
                self._state.command = CommandState()
            else:
                self._state.mockEnabled = False
                self._state.serialPort = ""
                self._state.command = CommandState()
            return self.snapshot()

    def _disconnect_locked(self, send_stop: bool) -> None:
        ser = self._serial
        if ser is not None and ser.is_open and send_stop:
            try:
                ser.write(b"0,0,0,90\n")
            except Exception:
                pass

        self._stop_reader.set()
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        self._serial = None
        self._state.connected = False
        self._state.serialPort = ""
        self._state.command = CommandState()

    def send_command(self, payload: dict[str, Any]) -> BridgeState:
        with self._lock:
            if (self._serial is None or not self._serial.is_open) and not self._state.mockEnabled:
                raise RuntimeError("Serial port is not connected")

            command = CommandState(
                leftSpeed=clamp(int(payload.get("leftSpeed", 0)), -255, 255),
                rightSpeed=clamp(int(payload.get("rightSpeed", 0)), -255, 255),
                winchSpeed=clamp(int(payload.get("winchSpeed", 0)), -255, 255),
                radarAngle=clamp(int(payload.get("radarAngle", 90)), 0, 180),
            )

            line = (
                f"{command.leftSpeed},{command.rightSpeed},"
                f"{command.winchSpeed},{command.radarAngle}\n"
            )
            if self._serial is not None and self._serial.is_open:
                self._serial.write(line.encode("ascii"))
            self._state.command = command
            self._state.lastError = ""
            return self.snapshot()

    def snapshot(self) -> BridgeState:
        with self._lock:
            return BridgeState(
                connected=self._state.connected,
                mockEnabled=self._state.mockEnabled,
                serialPort=self._state.serialPort,
                baudRate=self._state.baudRate,
                lastError=self._state.lastError,
                lastMessageAt=self._state.lastMessageAt,
                telemetry=Telemetry(**asdict(self._state.telemetry)),
                command=CommandState(**asdict(self._state.command)),
            )

    def _reader_loop(self) -> None:
        while not self._stop_reader.is_set():
            ser = self._serial
            if ser is None:
                return

            try:
                raw = ser.readline()
            except Exception as exc:
                with self._lock:
                    self._state.lastError = f"Serial read failed: {exc}"
                    self._state.connected = False
                    self._serial = None
                return

            if not raw:
                continue

            line = raw.decode(errors="replace").strip()
            if not line:
                continue

            parts = line.split(",")
            with self._lock:
                self._state.lastMessageAt = time.time()
                if len(parts) != 5:
                    self._state.lastError = line
                    continue

                try:
                    values = [int(part) for part in parts]
                except ValueError:
                    self._state.lastError = line
                    continue

                self._state.telemetry = Telemetry(
                    usRadar=values[0],
                    usFront=values[1],
                    usLeft=values[2],
                    usRight=values[3],
                    radarAngle=values[4],
                )
                self._state.lastError = ""

    def _mock_loop(self) -> None:
        while True:
            time.sleep(0.1)
            with self._lock:
                if not self._state.mockEnabled:
                    continue

                now = time.time()
                radar_angle = clamp(self._state.command.radarAngle, 0, 180)
                radar_phase = math.radians(radar_angle)

                us_radar = int(55 + 150 * (0.5 + 0.5 * math.sin(now * 1.7 + radar_phase * 2.0)))
                us_front = int(90 + 45 * math.sin(now * 0.55 + 0.3))
                us_left = int(100 + 55 * math.sin(now * 0.44 + 1.8))
                us_right = int(100 + 55 * math.sin(now * 0.44 - 1.8))

                self._state.telemetry = Telemetry(
                    usRadar=clamp(us_radar, 20, 340),
                    usFront=clamp(us_front, 20, 340),
                    usLeft=clamp(us_left, 20, 340),
                    usRight=clamp(us_right, 20, 340),
                    radarAngle=radar_angle,
                )
                self._state.lastMessageAt = now
                self._state.connected = False


BRIDGE = SerialBridge()


class ControlRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/state":
            self._send_json({"state": asdict(BRIDGE.snapshot())})
            return

        if self.path == "/api/ports":
            self._send_json({"ports": BRIDGE.list_ports()})
            return

        self._serve_static()

    def do_POST(self) -> None:
        try:
            body = self._read_json_body()
            if self.path == "/api/connect":
                port_name = str(body.get("port", "")).strip()
                if not port_name:
                    self._send_json({"error": "Missing port"}, status=HTTPStatus.BAD_REQUEST)
                    return
                state = BRIDGE.connect(port_name, int(body.get("baudRate", 115200)))
                self._send_json({"state": asdict(state)})
                return

            if self.path == "/api/disconnect":
                self._send_json({"state": asdict(BRIDGE.disconnect())})
                return

            if self.path == "/api/mock":
                enabled = bool(body.get("enabled", False))
                self._send_json({"state": asdict(BRIDGE.set_mock_enabled(enabled))})
                return

            if self.path == "/api/command":
                self._send_json({"state": asdict(BRIDGE.send_command(body))})
                return

            self._send_json({"error": "Not found"}, status=HTTPStatus.NOT_FOUND)
        except RuntimeError as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.CONFLICT)
        except serial.SerialException as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._send_json({"error": str(exc)}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _serve_static(self) -> None:
        target = self.path.split("?", 1)[0]
        if target in ("/", ""):
            file_path = WEB_DIR / "index.html"
        else:
            file_path = (WEB_DIR / target.lstrip("/")).resolve()

        if not str(file_path).startswith(str(WEB_DIR.resolve())) or not file_path.exists() or not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return

        content_type = mimetypes.guess_type(str(file_path))[0] or "application/octet-stream"
        raw = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)


def main() -> None:
    parser = argparse.ArgumentParser(description="Boat control HTTP server")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), ControlRequestHandler)
    print(f"Serving control UI on http://{args.host}:{args.port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        BRIDGE.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()