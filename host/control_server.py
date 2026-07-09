from __future__ import annotations

import argparse
import json
import math
import mimetypes
import os
import queue
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

# How often the server re-sends the last command over serial while connected.
# Must be shorter than the Arduino's FAILSAFE_TIMEOUT_MS (800ms) so that a
# stationary boat under active control is never mistaken for a lost link.
HEARTBEAT_INTERVAL_S = 0.25


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


@dataclass
class Telemetry:
    usRadar: int | None = None
    usFront: int | None = None
    usLeft: int | None = None
    usRight: int | None = None
    radarAngle: int = 90
    boatHeadingDeg: int = 0
    boatX: float = 0.0
    boatY: float = 0.0


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
        self._lock = threading.RLock()
        self._serial: serial.Serial | None = None
        self._reader_thread: threading.Thread | None = None
        self._stop_reader = threading.Event()
        self._mock_thread = threading.Thread(target=self._mock_loop, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._state = BridgeState()

        # SSE fan-out: every connected stream client owns a bounded queue that
        # receives a fresh state snapshot whenever telemetry or state changes.
        self._subscribers: set[queue.Queue] = set()
        self._sub_lock = threading.Lock()
        
        # Simulation state
        self._sim_boat_x = 0.0
        self._sim_boat_y = 0.0
        self._sim_boat_heading_rad = 0.0
        self._sim_boat_speed_cms = 0.0
        self._sim_targets: list[dict] = []
        self._sim_last_ts = time.time()
        # Rectangular boundary that exists in the mock world (half extents in cm)
        self._sim_bounds_half_x = 600.0
        self._sim_bounds_half_y = 450.0
        # Active mock scenario + its start/goal points (world coords, cm).
        self._sim_scenario = "serpentine"
        self._sim_start: dict | None = None
        self._sim_goal: dict | None = None

        self._mock_thread.start()
        self._heartbeat_thread.start()

    def list_ports(self) -> list[str]:
        return [port.device for port in list_ports.comports()]

    # --- SSE fan-out -----------------------------------------------------

    def subscribe(self) -> "queue.Queue":
        """Register a new stream client and return its snapshot queue."""
        q: queue.Queue = queue.Queue(maxsize=8)
        with self._sub_lock:
            self._subscribers.add(q)
        return q

    def unsubscribe(self, q: "queue.Queue") -> None:
        with self._sub_lock:
            self._subscribers.discard(q)

    def _publish(self) -> None:
        """Push the current snapshot to every subscriber (drop-oldest on full)."""
        data = asdict(self.snapshot())
        with self._sub_lock:
            for q in self._subscribers:
                try:
                    q.put_nowait(data)
                except queue.Full:
                    # Slow client: discard its stalest frame and keep the newest.
                    try:
                        q.get_nowait()
                        q.put_nowait(data)
                    except (queue.Empty, queue.Full):
                        pass

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
            self._publish()
            return self.snapshot()

    def disconnect(self) -> BridgeState:
        with self._lock:
            self._disconnect_locked(send_stop=True)
            self._state.mockEnabled = False
            self._publish()
            return self.snapshot()

    def set_mock_enabled(self, enabled: bool, scenario: str = "serpentine") -> BridgeState:
        with self._lock:
            if enabled:
                self._disconnect_locked(send_stop=True)
                self._state.mockEnabled = True
                self._state.serialPort = "MOCK"
                self._state.lastError = ""
                self._state.lastMessageAt = time.time()
                self._state.command = CommandState()
                self._sim_scenario = scenario
                # Build the world and drop the boat at the scenario's start pose.
                if scenario == "serpentine":
                    # Boustrophedon "maze": three baffle walls with alternating
                    # gaps. Boat starts bottom-left, goal is the top-right corner,
                    # so it must weave right-up-left-up-right to cross.
                    self._sim_targets = self._create_serpentine_targets()
                    self._sim_boat_x = -(self._sim_bounds_half_x - 80)
                    self._sim_boat_y = -(self._sim_bounds_half_y - 80)
                    self._sim_boat_heading_rad = 0.0
                    self._sim_goal = {
                        "x": self._sim_bounds_half_x - 80,
                        "y": self._sim_bounds_half_y - 80,
                    }
                else:
                    self._sim_targets = self._create_random_targets(24)
                    self._sim_boat_x = 0.0
                    self._sim_boat_y = 0.0
                    self._sim_boat_heading_rad = 0.0
                    self._sim_goal = None
                self._sim_start = {"x": self._sim_boat_x, "y": self._sim_boat_y}
                self._sim_boat_speed_cms = 0.0
                self._sim_last_ts = time.time()
                self._state.telemetry = Telemetry(
                    usRadar=180, usFront=120, usLeft=95, usRight=105, radarAngle=90,
                    boatX=self._sim_boat_x, boatY=self._sim_boat_y,
                    boatHeadingDeg=int(math.degrees(self._sim_boat_heading_rad)) % 360,
                )
            else:
                self._state.mockEnabled = False
                self._state.serialPort = ""
                self._state.command = CommandState()
            self._publish()
            return self.snapshot()

    def world_snapshot(self) -> dict:
        """Ground-truth mock world: obstacle bodies, boundary and sensor range.

        Only meaningful while mock mode is on; lets the UI draw the real world
        next to the radar picture for side-by-side comparison.
        """
        with self._lock:
            return {
                "mockEnabled": self._state.mockEnabled,
                "boundsHalfX": self._sim_bounds_half_x,
                "boundsHalfY": self._sim_bounds_half_y,
                "maxRange": 450,
                "targets": [dict(t) for t in self._sim_targets],
                "scenario": self._sim_scenario,
                "start": dict(self._sim_start) if self._sim_start else None,
                "goal": dict(self._sim_goal) if self._sim_goal else None,
            }

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
            self._publish()
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
                    boatX=self._sim_boat_x,
                    boatY=self._sim_boat_y,
                )
                self._state.lastError = ""
                self._publish()

    def _heartbeat_loop(self) -> None:
        """Periodically re-send the last command so the boat's failsafe stays armed.

        The Arduino stops its motors if no valid packet arrives within its
        failsafe window. Without a heartbeat, a boat that is told "go forward"
        and then left alone would be halted by the watchdog. Re-transmitting the
        current command keeps the RF link alive so the boat keeps doing exactly
        what the UI last asked, while a genuine link loss still trips the failsafe.
        """
        while True:
            time.sleep(HEARTBEAT_INTERVAL_S)
            with self._lock:
                ser = self._serial
                if ser is None or not ser.is_open:
                    continue
                command = self._state.command
                line = (
                    f"{command.leftSpeed},{command.rightSpeed},"
                    f"{command.winchSpeed},{command.radarAngle}\n"
                )
                try:
                    ser.write(line.encode("ascii"))
                except Exception:
                    pass

    def _mock_loop(self) -> None:
        import random
        
        while True:
            time.sleep(0.05)  # 50ms = 20Hz sampling
            with self._lock:
                if not self._state.mockEnabled:
                    continue
                
                now = time.time()
                dt = max(0.01, now - self._sim_last_ts)
                self._sim_last_ts = now
                
                # Update boat dynamics from joystick commands
                # Throttle: average of left and right speeds (0-1 range)
                throttle = (self._state.command.leftSpeed + self._state.command.rightSpeed) / (2 * 255)
                # Turn: difference of left and right speeds (rotation)
                turn = (self._state.command.rightSpeed - self._state.command.leftSpeed) / (2 * 255)
                
                # Update boat speed (smooth acceleration)
                target_speed = throttle * 45  # Max 45 cm/s (physical hull limit)
                self._sim_boat_speed_cms += (target_speed - self._sim_boat_speed_cms) * 0.24 * dt / 0.05
                
                # Update boat heading based on turn
                self._sim_boat_heading_rad += turn * (math.pi * 0.25) * dt
                # Normalize angle to [0, 2π)
                while self._sim_boat_heading_rad < 0:
                    self._sim_boat_heading_rad += 2 * math.pi
                while self._sim_boat_heading_rad >= 2 * math.pi:
                    self._sim_boat_heading_rad -= 2 * math.pi
                
                # Update boat position based on velocity and heading
                self._sim_boat_x += math.sin(self._sim_boat_heading_rad) * self._sim_boat_speed_cms * dt
                self._sim_boat_y += math.cos(self._sim_boat_heading_rad) * self._sim_boat_speed_cms * dt
                
                # Keep the boat inside the rectangular boundary
                self._sim_boat_x = clamp(self._sim_boat_x, -self._sim_bounds_half_x, self._sim_bounds_half_x)
                self._sim_boat_y = clamp(self._sim_boat_y, -self._sim_bounds_half_y, self._sim_bounds_half_y)
                
                # Sensor model mirrors the real boat hardware exactly:
                #   * 4 ultrasonic sensors mounted 90° apart on ONE servo axis.
                #   * They all rotate together as the servo turns, so a sweep of
                #     just 0..90° already covers the full 360° around the boat.
                #   * radarAngle is the servo position, commanded by the PC and
                #     echoed back; at angle 0 the sensors point F / R / B / L.
                max_range = 450
                fov_deg = 15
                boat_heading_deg = math.degrees(self._sim_boat_heading_rad)

                # The servo follows the last PC command (the firmware just echoes
                # cmd.radarAngle). No sweep is invented here; the PC drives it.
                sweep = self._state.command.radarAngle

                us_front = self._cast_sensor_ray(boat_heading_deg + sweep + 0, fov_deg, max_range)
                us_right = self._cast_sensor_ray(boat_heading_deg + sweep + 90, fov_deg, max_range)
                us_back = self._cast_sensor_ray(boat_heading_deg + sweep + 180, fov_deg, max_range)
                us_left = self._cast_sensor_ray(boat_heading_deg + sweep + 270, fov_deg, max_range)

                # Telemetry matches the 5-field serial protocol; the extra boat
                # pose fields are simulation-only helpers (not on the real wire).
                # usRadar carries the rear-facing sensor at servo home (0°).
                self._state.telemetry = Telemetry(
                    usRadar=us_back,
                    usFront=us_front,
                    usLeft=us_left,
                    usRight=us_right,
                    radarAngle=sweep,
                    boatHeadingDeg=int(boat_heading_deg) % 360,
                    boatX=self._sim_boat_x,
                    boatY=self._sim_boat_y,
                )
                self._state.lastMessageAt = now
                self._state.connected = False
            self._publish()

    def _create_serpentine_targets(self) -> list[dict]:
        """Three baffle walls forming a boustrophedon course (see the sketch).

        Each wall is a dense row of overlapping circles so the ultrasonic rays
        see a continuous barrier. The gaps alternate side to side, so the only
        way from the bottom-left start to the top-right goal is to weave:
        right -> up -> left -> up -> right -> up.
        """
        hx = self._sim_bounds_half_x
        radius = 26.0
        spacing = 34.0

        def add_wall(targets: list[dict], y: float, x_from: float, x_to: float) -> None:
            step = spacing if x_to >= x_from else -spacing
            n = int(abs(x_to - x_from) / spacing)
            for i in range(n + 1):
                xi = x_from + step * i
                targets.append({"x": float(xi), "y": float(y), "radius": radius})
            targets.append({"x": float(x_to), "y": float(y), "radius": radius})

        targets: list[dict] = []
        # y splits the pool into 4 lanes (~225 cm tall each).
        add_wall(targets, -225.0, -hx, 230.0)   # lower divider: wall on LEFT,  gap RIGHT
        add_wall(targets, 0.0, hx, -230.0)       # middle divider: wall on RIGHT, gap LEFT
        add_wall(targets, 225.0, -hx, 230.0)     # upper divider: wall on LEFT,  gap RIGHT
        return targets

    def _create_random_targets(self, count: int) -> list[dict]:
        import random
        targets = []
        margin = 40  # keep bodies away from the walls
        min_x = -self._sim_bounds_half_x + margin
        max_x = self._sim_bounds_half_x - margin
        min_y = -self._sim_bounds_half_y + margin
        max_y = self._sim_bounds_half_y - margin
        # Keep a clear circle around the boat's start point (0, 0). The boat is
        # dropped into open water, so a body must never spawn on top of it —
        # otherwise the boat begins already inside an obstacle, which no
        # controller could avoid.
        start_clear = 110  # cm clear radius around the origin
        for i in range(count):
            for _attempt in range(50):
                x = random.uniform(min_x, max_x)
                y = random.uniform(min_y, max_y)
                radius = 14 + random.random() * 26
                if math.hypot(x, y) - radius >= start_clear:
                    break
            targets.append({'x': x, 'y': y, 'radius': radius})
        return targets

    def _cast_wall_distance(self, center_rad: float, max_range: int) -> float:
        """Distance from boat to the rectangular boundary along a direction."""
        sin_t = math.sin(center_rad)
        cos_t = math.cos(center_rad)
        half_x = self._sim_bounds_half_x
        half_y = self._sim_bounds_half_y
        bx = self._sim_boat_x
        by = self._sim_boat_y
        nearest = float('inf')
        eps = 1e-6

        # Vertical walls: x = +/- half_x
        if abs(sin_t) > eps:
            for wx in (-half_x, half_x):
                t = (wx - bx) / sin_t
                if t > 0:
                    y_hit = by + t * cos_t
                    if -half_y - eps <= y_hit <= half_y + eps and t < nearest:
                        nearest = t
        # Horizontal walls: y = +/- half_y
        if abs(cos_t) > eps:
            for wy in (-half_y, half_y):
                t = (wy - by) / cos_t
                if t > 0:
                    x_hit = bx + t * sin_t
                    if -half_x - eps <= x_hit <= half_x + eps and t < nearest:
                        nearest = t
        return nearest

    def _cast_sensor_ray(self, center_deg: float, fov_deg: float, max_range: int) -> int:
        """Cast a sensor ray and find nearest obstacle within FOV."""
        nearest = max_range + 1

        # Boundary walls (exist in the mock world, detected like any obstacle)
        wall_dist = self._cast_wall_distance(math.radians(center_deg), max_range)
        if wall_dist <= max_range:
            nearest = wall_dist

        for target in self._sim_targets:
            dx = target['x'] - self._sim_boat_x
            dy = target['y'] - self._sim_boat_y

            # Distance from boat to target
            dist = math.hypot(dx, dy)

            # Skip if too far
            if dist > max_range + target['radius']:
                continue

            # Angle to target (normalized [0, 360))
            target_deg = math.degrees(math.atan2(dx, dy)) % 360

            # Shortest angular distance — works even when center_deg is unnormalized
            # (e.g. 440° for heading 350° + sweep 90°). The naive abs() approach
            # produces a negative result when the raw diff exceeds 360, letting
            # out-of-FOV targets pass the check.
            angle_diff = abs((center_deg - target_deg + 180) % 360 - 180)

            # A target is not a point: it subtends an angular half-width of
            # asin(radius/dist) as seen from the boat. A close body fills a wide
            # slice of the beam cone, so its SURFACE can be inside the FOV even
            # when its CENTER is well off-axis. Comparing only the center bearing
            # (as before) made nearby obstacles invisible unless the servo pointed
            # almost exactly at them — they were missed as the boat passed by.
            # Real ultrasonic cones echo off the nearest surface anywhere in the
            # cone, so subtract the target's angular radius before the FOV test.
            angular_radius = math.degrees(math.asin(min(1.0, target['radius'] / dist))) if dist > 1e-6 else 90.0

            if angle_diff - angular_radius > fov_deg / 2:
                continue
            
            # Impact distance: surface distance minus radius
            impact = max(20, dist - target['radius'])
            
            if impact < nearest:
                nearest = impact
        
        return int(round(nearest)) if nearest <= max_range else 999


BRIDGE = SerialBridge()


class ControlRequestHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/api/state":
            self._send_json({"state": asdict(BRIDGE.snapshot())})
            return

        if self.path == "/api/stream":
            self._serve_sse()
            return

        if self.path == "/api/ports":
            self._send_json({"ports": BRIDGE.list_ports()})
            return

        if self.path == "/api/world":
            self._send_json(BRIDGE.world_snapshot())
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
                scenario = str(body.get("scenario", "serpentine"))
                self._send_json({"state": asdict(BRIDGE.set_mock_enabled(enabled, scenario))})
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

    def _serve_sse(self) -> None:
        """Server-Sent Events stream: pushes a state snapshot on every change.

        Replaces slow client polling — the reader/mock loops publish a fresh
        snapshot the instant new telemetry is parsed, so the UI updates in
        near real time. A periodic comment line keeps idle connections alive.
        """
        q = BRIDGE.subscribe()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            # Send the current state immediately so a fresh client isn't blank
            # until the next telemetry frame arrives.
            self._sse_send(asdict(BRIDGE.snapshot()))
            while True:
                try:
                    data = q.get(timeout=15)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
                    continue
                self._sse_send(data)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return
        finally:
            BRIDGE.unsubscribe(q)

    def _sse_send(self, state: dict[str, Any]) -> None:
        raw = json.dumps({"state": state}).encode("utf-8")
        self.wfile.write(b"data: " + raw + b"\n\n")
        self.wfile.flush()

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw.decode("utf-8"))

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        raw = json.dumps(payload).encode("utf-8")
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return

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
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
            return


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