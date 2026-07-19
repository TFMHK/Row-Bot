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
from urllib.parse import unquote

import serial
from serial.tools import list_ports


ROOT_DIR = Path(__file__).resolve().parent
WEB_DIR = ROOT_DIR / "web"
LOG_DIR = ROOT_DIR / "logs"

# --- Autonomous-run data collection ----------------------------------------
# The browser navigator POSTs batches of decision records (telemetry it RECEIVES
# + commands it SENDS + what it perceived) to /api/navlog. We append them as
# newline-delimited JSON to a per-run file so a run can be replayed/analysed
# afterwards even if the browser (phone) is closed mid-experiment.
_navlog_lock = threading.Lock()
_navlog_file: Path | None = None


def navlog_append(records: list, reset: bool = False) -> str:
    """Append decision records to the current run's NDJSON log (thread-safe).

    `reset` starts a fresh file (call it once when a new autonomous run arms).
    """
    global _navlog_file
    with _navlog_lock:
        if reset or _navlog_file is None:
            LOG_DIR.mkdir(exist_ok=True)
            ts = time.strftime("%Y%m%d-%H%M%S")
            _navlog_file = LOG_DIR / f"nav-{ts}.ndjson"
        with _navlog_file.open("a", encoding="utf-8") as fh:
            for rec in records:
                fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
        return _navlog_file.name


def list_navlogs() -> list:
    """Return recorded run logs (newest first) with byte size + record count.

    Used by the UI's replay picker so the operator can choose a recorded sail
    to physically play back.
    """
    if not LOG_DIR.exists():
        return []
    out = []
    for p in sorted(LOG_DIR.glob("nav-*.ndjson"), key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            st = p.stat()
            with p.open("r", encoding="utf-8") as fh:
                records = sum(1 for line in fh if line.strip())
        except OSError:
            continue
        out.append({
            "name": p.name,
            "size": st.st_size,
            "mtime": st.st_mtime,
            "records": records,
        })
    return out


# How often the server re-sends the last command over serial while connected.
# Must be shorter than the Arduino's FAILSAFE_TIMEOUT_MS (1500ms) so that a
# stationary boat under active control is never mistaken for a lost link.
# Raised 0.25 -> 0.40s: at 2000bps a full command+telemetry round-trip takes
# ~300ms on air. A 250ms heartbeat sent the next command before the boat's
# reply finished -> half-duplex collision -> both frames lost (observed: ~0.5
# telemetry/s at close range). 400ms comfortably fits the round-trip.
HEARTBEAT_INTERVAL_S = 0.40
# How often the shore re-sends the nav-config frame so a boat that just rebooted
# (or was armed late) still picks up the current UI tuning without a fresh flash.
NAVCFG_RESEND_INTERVAL_S = 2.0

# --- Real-world link resilience --------------------------------------------
# Telemetry is considered STALE if no fresh frame arrived within this window.
# The reader updates lastMessageAt on every received line, so a healthy link
# (even at ~1 Hz) stays well under this. Exposed to the UI via BridgeState.stale.
TELEMETRY_STALE_S = 3.0
# Minimum spacing between PHYSICAL serial writes. A burst of UI commands can
# otherwise saturate the half-duplex RF link and starve the boat's return
# telemetry (observed: the stream freezes under command flooding). A skipped
# command still becomes state.command and is pushed out by the heartbeat, so
# nothing is lost.
SERIAL_MIN_WRITE_INTERVAL_S = 0.1
# When the link goes stale the watchdog reopens the port ONCE per cooldown
# (reopening toggles DTR and resets the radio bridge, reviving the stream).
# The cooldown must exceed connect()'s 2 s settle so we never thrash the boot
# (observed: fast repeated reconnects never let the boat finish booting).
RECONNECT_COOLDOWN_S = 6.0
# Median-filter window per sensor channel to reject single-frame spikes (real
# ultrasonic readings jitter and occasionally return phantom short/long values).
SENSOR_MEDIAN_WINDOW = 3
# While the boat is stopped its radar servo would sit still and stop pinging,
# so telemetry only flows during motion. When idle we advance radarAngle through
# these steps on each heartbeat to keep the servo scanning and pings arriving.
IDLE_SWEEP_ANGLES = (0, 30, 60, 90, 120, 150, 180)


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


# Drive-motor speed by ABSOLUTE value, applied to BOTH manual and autonomous
# commands right before they reach the motor (build_serial_line + the mock loop):
#   |v| < 35          -> 0   (dead zone: too weak to bother -> motor off)
#   otherwise         -> clamped into the usable [70, 100] band (continuous)
# The user's control scale is this 70..100 band; the sign (direction) is preserved.
MOTOR_DEADZONE = 35        # below this magnitude -> 0 (off)
MOTOR_LOW_SPEED = 70       # bottom of the usable band
MOTOR_HIGH_SPEED = 100     # top of the usable band
# Upper bound the operator may enter for each motor's absolute output speed in
# the UI. Kept ABOVE MOTOR_HIGH_SPEED (the shaping band used by nav/mock) so the
# operator can push a motor harder than the autonomous band without affecting
# nav/mock physics. PWM is 8-bit (0..255) so 150 is well within range.
MOTOR_ABS_MAX = 150
# Absolute output speed (magnitude) each PHYSICAL drive motor runs at whenever
# it is driving. The shaped command still decides OFF (0) vs on and the
# direction (sign); this only overrides the magnitude, so the operator can tune
# each motor's absolute speed live from the UI to balance a weaker motor without
# reflashing the boat (applied host-side in build_serial_line; the logical
# CommandState is unchanged). Default calibration: left motor 84, right motor
# 88. Clamped to [0, MOTOR_HIGH_SPEED].
MOTOR_LEFT_ABS_DEFAULT = 84
MOTOR_RIGHT_ABS_DEFAULT = 88

# Ultrasonic readings closer than this are treated as spurious (echo ring-down,
# hull reflections, sensor cross-talk) and are discarded — mapped to the
# out-of-range sentinel so nav and the radar display ignore them.
MIN_VALID_DISTANCE_CM = 10
OUT_OF_RANGE_CM = 999

# --- Mock-hull dynamics (a real boat is NOT a kinematic point) ---------------
# The vessel carries momentum: thrust from the motors fights water drag, so it
# keeps gliding after the throttle is cut and needs time to spin up/down. These
# constants are shared verbatim with the client odometry (integratePose in
# app.js) so the dead-reckoned pose stays glued to the simulated ground truth.
MOCK_MAX_SPEED_CMS = 45.0          # steady-state hull speed at full throttle
MOCK_TURN_RATE = math.pi * 0.25    # steady-state yaw rate at full differential
MOCK_LINEAR_DRAG = 1.4             # 1/s linear drag -> ~0.7 s coast time-const
MOCK_TURN_DRAG = 2.5               # 1/s yaw drag    -> ~0.4 s spin time-const

# --- Mock ultrasonic noise (real hardware is not perfect) --------------------
# Every so often a ping returns nonsense: no echo at all (dropout) or a phantom
# short reading (cross-talk / a wave). Every good ping still jitters a few cm.
MOCK_SENSOR_DROPOUT_P = 0.03       # P(reading -> no echo / 999)
MOCK_SENSOR_SPIKE_P = 0.03         # P(reading -> phantom short spike)
MOCK_SENSOR_JITTER_CM = 2.0        # 1-sigma gaussian jitter on good pings

# Baffle / boundary obstacles are thin STRAIGHT walls, not blobs. A physical
# baffle is a few centimetres thick; model it as a 5 cm band.
MOCK_WALL_THICKNESS_CM = 5.0

# Servo-to-bow offset (degrees): on the REAL boat the servo home (angle 0) is
# NOT aligned with the bow — the bow sits at servo ~60°, so the onboard nav (and
# its mock twin) recover a sensor's bow-relative bearing as (base + servo - 60).
# The mock MUST bake in the SAME offset so servo 60 => front sensor points at the
# bow, exactly like the water; otherwise the simulated geometry is rotated 60°
# from what the navigator assumes and "works in mock" no longer implies "works on
# water". Keep in sync with NAV_BOW_OFFSET_DEG in src/main.cpp and
# BOW_SERVO_OFFSET_DEG in app.js.
MOCK_BOW_OFFSET_DEG = 60.0


def filter_near_reading(value: int) -> int:
    """Drop sensor points closer than MIN_VALID_DISTANCE_CM (return sentinel)."""
    return value if value >= MIN_VALID_DISTANCE_CM else OUT_OF_RANGE_CM


def shape_motor_speed(value: int) -> int:
    """Shape a drive-motor speed to {0} ∪ ±[70, 100], preserving direction.

    - A magnitude below MOTOR_DEADZONE (35) becomes 0 (motor off).
    - Any larger magnitude is clamped into the usable [70, 100] band, so the
      operator gets continuous control across that range.
    """
    magnitude = abs(value)
    if magnitude < MOTOR_DEADZONE:
        return 0
    stepped = min(max(magnitude, MOTOR_LOW_SPEED), MOTOR_HIGH_SPEED)
    return stepped if value > 0 else -stepped


def motor_direction(value: int) -> int:
    """Return a drive motor's direction: +1 ahead, -1 astern, 0 stopped.

    Mirrors shape_motor_speed's dead zone. On the real boat every running motor
    turns at its fixed per-motor calibration speed (build_serial_line overrides
    the magnitude), so only each motor's DIRECTION affects how the hull moves —
    the joystick just picks forward / reverse / stop per motor.
    """
    shaped = shape_motor_speed(value)
    if shaped > 0:
        return 1
    if shaped < 0:
        return -1
    return 0


def build_nav_config_line(nc: "NavConfig") -> str:
    """Serialize the onboard-nav tuning into the shore's 'N,...' line. The shore
    relay parses this, packs it into a signed RfNavConfig frame and forwards it
    to the boat over RF, so all nav calibration is done from the UI (no reflash).
    Field order MUST match the shore sscanf + the firmware decode."""
    return (
        f"N,{nc.frontBlock},{nc.frontClear},{nc.frontEmergency},"
        f"{nc.decision},{nc.decisionHalf},{nc.sideStandoff},"
        f"{nc.bowOffset},{nc.sweepSign}\n"
    )


def build_serial_line(
    command: "CommandState",
    motor_left_abs: int = MOTOR_LEFT_ABS_DEFAULT,
    motor_right_abs: int = MOTOR_RIGHT_ABS_DEFAULT,
) -> str:
    """Build the outgoing serial packet, applying hardware wiring corrections.

    These fixes live here (host side) so the boat firmware never needs
    re-flashing:
    - The two drive motor channels map STRAIGHT THROUGH (logical left -> physical
      left slot, logical right -> right slot). They used to be swapped on the
      assumption the hardware wiring was reversed, but field observation showed
      the autonomous TURN went the wrong way while forward/back were correct —
      the signature of an inverted turn only — so the swap was removed. If the
      turn direction is ever found inverted again, swap the two lines below.
    - The net winch motor is wired with reversed polarity, so its speed is
      inverted.
    The drive motor speeds are shaped via shape_motor_speed() so each wheel is
    either off (0) or driving, then each driving motor's magnitude is set to its
    operator-tuned absolute speed (motor_left_abs / motor_right_abs).
    The logical CommandState kept in the bridge state stays unchanged, so the
    UI and mock trajectory still see left/right/winch in their intended sense.
    """
    # AUTONOMOUS mode: the boat runs the navigator ONBOARD and decides each
    # motor's DIRECTION locally (forward / turn / reverse). The shore only sets
    # the per-motor POWER — so when we switch to auto we send the operator-tuned
    # absolute speeds (positive) as the left/right fields, and the boat reads
    # their MAGNITUDE as its autonomous drive power. The incoming leftSpeed/
    # rightSpeed (0 from the UI in auto) are ignored here. This keeps motor
    # calibration entirely on the shore — no reflash needed to retune it.
    if command.mode == 1:
        powerLeft = clamp(int(motor_left_abs), 0, MOTOR_ABS_MAX)
        powerRight = clamp(int(motor_right_abs), 0, MOTOR_ABS_MAX)
        physicalWinch = -command.winchSpeed
        return (
            f"{powerLeft},{powerRight},"
            f"{physicalWinch},{command.radarAngle},{command.mode}\n"
        )

    # Straight-through: logical left -> physical left slot, logical right -> right.
    physicalLeft = shape_motor_speed(command.leftSpeed)
    physicalRight = shape_motor_speed(command.rightSpeed)
    # Override each driving motor's magnitude with its operator-set absolute
    # speed. motor_left_abs governs the physical left motor, motor_right_abs the
    # physical right motor. An already-off motor (0) stays off; the sign/direction
    # is preserved.
    if physicalLeft != 0:
        mag = clamp(int(motor_left_abs), 0, MOTOR_ABS_MAX)
        physicalLeft = mag if physicalLeft > 0 else -mag
    if physicalRight != 0:
        mag = clamp(int(motor_right_abs), 0, MOTOR_ABS_MAX)
        physicalRight = mag if physicalRight > 0 else -mag
    physicalWinch = -command.winchSpeed
    return (
        f"{physicalLeft},{physicalRight},"
        f"{physicalWinch},{command.radarAngle},{command.mode}\n"
    )


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
    mode: int = 0  # 0 = ידני (ירוק), 1 = אוטומטי (כחול) — צבע טבעת הלדים בסירה


@dataclass
class NavConfig:
    """Onboard-nav tuning params, shore-controlled from the UI. Applied live to
    the mock twin; sent to the boat over RF when auto is armed (once flashed)."""
    frontBlock: int = 55      # front < this => avoidance turn
    frontClear: int = 90      # spin until front >= this (hysteresis release)
    frontEmergency: int = 45  # front < this => committed reverse (drift safety)
    decision: int = 100       # start deciding the turn this far out (early)
    decisionHalf: int = 60    # half of the forward decision arc (=120 deg total)
    sideStandoff: int = 40    # keep this clearance from side walls when passing
    bowOffset: int = 60       # bow-vs-servo mounting offset (deg) — land calibration
    sweepSign: int = 1        # radar sweep / turn polarity: +1 or -1 (land calibration)


@dataclass
class BridgeState:
    connected: bool = False
    mockEnabled: bool = False
    serialPort: str = ""
    baudRate: int = 115200
    lastError: str = ""
    lastMessageAt: float | None = None
    stale: bool = False
    motorLeftAbs: int = MOTOR_LEFT_ABS_DEFAULT
    motorRightAbs: int = MOTOR_RIGHT_ABS_DEFAULT
    navConfig: NavConfig = field(default_factory=NavConfig)
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
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._state = BridgeState()
        # Wall-clock of the last command actually written to the serial port
        # (from send_command OR the heartbeat). The heartbeat only retransmits
        # when this is stale, so an actively-commanding UI never doubles the RF
        # traffic — critical on the half-duplex link, where every extra shore TX
        # is a window in which the boat's return telemetry collides and is lost.
        self._last_serial_write_ts = 0.0
        # Wall-clock of the last nav-config frame sent to the shore (throttles the
        # periodic resend in the heartbeat; reset on connect to push it promptly).
        self._last_navcfg_write_ts = 0.0
        # Last successfully-connected port/baud, so the watchdog can reopen the
        # link on its own when telemetry goes stale (real RF bridges hang).
        self._last_port: str = ""
        self._last_baud: int = 115200
        self._last_reconnect_ts = 0.0
        # Rolling per-channel history for median spike rejection on live sensors.
        self._sensor_hist: dict[str, list[int]] = {
            "usRadar": [], "usFront": [], "usLeft": [], "usRight": [],
        }
        # Index into IDLE_SWEEP_ANGLES used to keep the radar scanning at rest.
        self._idle_sweep_i = 0

        # SSE fan-out: every connected stream client owns a bounded queue that
        # receives a fresh state snapshot whenever telemetry or state changes.
        self._subscribers: set[queue.Queue] = set()
        self._sub_lock = threading.Lock()
        
        # Simulation state
        self._sim_boat_x = 0.0
        self._sim_boat_y = 0.0
        self._sim_boat_heading_rad = 0.0
        self._sim_boat_speed_cms = 0.0
        self._sim_boat_turn_rate = 0.0  # rad/s (yaw carries angular momentum)
        self._sim_targets: list[dict] = []
        self._sim_walls: list[dict] = []  # thin straight-line obstacles
        self._sim_last_ts = time.time()
        # Rectangular pool boundary in the mock world (half extents in cm).
        # Pool is 2.2 m wide (x) by 4.5 m long (y): the boat travels along the
        # 4.5 m length (bottom -> top) and the baffle gaps sit across the 2.2 m width.
        self._sim_bounds_half_x = 110.0
        self._sim_bounds_half_y = 225.0
        # Active mock scenario + its start/goal points (world coords, cm).
        self._sim_scenario = "rectangle"
        self._sim_start: dict | None = None
        self._sim_goal: dict | None = None

        self._mock_thread.start()
        self._heartbeat_thread.start()
        self._watchdog_thread.start()

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
            self._last_navcfg_write_ts = 0.0  # force a nav-config push on the next beat
            self._state.serialPort = port_name
            self._state.baudRate = baud_rate
            self._state.lastError = ""
            self._last_port = port_name
            self._last_baud = baud_rate
            self._last_reconnect_ts = time.time()
            # Grace period: mark data fresh at connect so the watchdog doesn't
            # immediately re-trigger while the boat is still booting/streaming.
            self._state.lastMessageAt = time.time()
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

    def set_mock_enabled(self, enabled: bool, scenario: str = "rectangle") -> BridgeState:
        with self._lock:
            if enabled:
                self._disconnect_locked(send_stop=True)
                self._state.mockEnabled = True
                self._state.serialPort = "MOCK"
                self._state.lastError = ""
                self._state.lastMessageAt = time.time()
                self._state.command = CommandState()
                self._sim_scenario = scenario
                self._sim_walls = []
                # Build the world and drop the boat at the scenario's start pose.
                if scenario == "rectangle":
                    # Rectangular pool with three baffle walls jutting from the
                    # side walls. The boat starts bottom-left and the goal is the
                    # top-left corner, so it must weave: up -> right (around the
                    # lower baffle) -> up -> left (middle baffle) -> up -> right
                    # (upper baffle) -> up.
                    self._sim_targets = self._create_rectangle_targets()
                    self._sim_boat_x = -(self._sim_bounds_half_x - 20)
                    self._sim_boat_y = -(self._sim_bounds_half_y - 40)
                    self._sim_boat_heading_rad = math.pi / 2  # פונה ימינה (+x), לא למעלה
                    self._sim_goal = {
                        "x": -(self._sim_bounds_half_x - 20),
                        "y": self._sim_bounds_half_y - 30,
                    }
                elif scenario == "serpentine":
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
                self._sim_boat_turn_rate = 0.0
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
                "walls": [
                    {k: w[k] for k in ("x1", "y1", "x2", "y2", "thickness")}
                    for w in self._sim_walls
                ],
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
                mode=clamp(int(payload.get("mode", self._state.command.mode)), 0, 1),
            )

            line = build_serial_line(
                command, self._state.motorLeftAbs, self._state.motorRightAbs
            )
            now = time.time()
            if self._serial is not None and self._serial.is_open:
                # Rate-limit PHYSICAL writes: a flood of UI commands can saturate
                # the half-duplex RF link and freeze the boat's return telemetry.
                # A skipped command still becomes state.command and is pushed out
                # by the heartbeat within HEARTBEAT_INTERVAL_S, so none is lost.
                if now - self._last_serial_write_ts >= SERIAL_MIN_WRITE_INTERVAL_S:
                    self._serial.write(line.encode("ascii"))
                    self._last_serial_write_ts = now
            self._state.command = command
            self._state.lastError = ""
            self._publish()
            return self.snapshot()

    def set_motor_config(self, payload: dict[str, Any]) -> BridgeState:
        """Update the per-motor absolute output speeds from the UI.

        Only the keys present in the payload are changed, each clamped to the
        [0, MOTOR_ABS_MAX] range. Applied to every subsequent serial packet
        (send_command + heartbeat) with no boat reflash.
        """
        with self._lock:
            if payload.get("motorLeftAbs") is not None:
                self._state.motorLeftAbs = clamp(
                    int(payload["motorLeftAbs"]), 0, MOTOR_ABS_MAX
                )
            if payload.get("motorRightAbs") is not None:
                self._state.motorRightAbs = clamp(
                    int(payload["motorRightAbs"]), 0, MOTOR_ABS_MAX
                )
            self._publish()
            return self.snapshot()

    def set_nav_config(self, payload: dict[str, Any]) -> BridgeState:
        """Update the shore-controlled onboard-nav tuning params. Each key is
        optional and clamped to a sane range. Broadcast so the mock twin (and,
        once flashed, the boat) picks up the new values live."""
        with self._lock:
            c = self._state.navConfig

            def upd(key: str, lo: int, hi: int) -> None:
                if payload.get(key) is not None:
                    setattr(c, key, clamp(int(payload[key]), lo, hi))

            upd("frontBlock", 10, 300)
            upd("frontClear", 10, 320)
            upd("frontEmergency", 5, 200)
            upd("decision", 20, 400)
            upd("decisionHalf", 10, 90)
            upd("sideStandoff", 5, 200)
            upd("bowOffset", 0, 180)
            upd("sweepSign", -1, 1)
            # Push the new tuning to the boat immediately (also periodically
            # resent by the heartbeat so a rebooted boat re-syncs on its own).
            self._send_nav_config_locked()
            self._publish()
            return self.snapshot()

    def _send_nav_config_locked(self) -> None:
        """Write the current nav-config 'N,...' line to the serial link. Assumes
        the bridge lock is held. No-op when the port is closed (mock)."""
        ser = self._serial
        if ser is None or not ser.is_open:
            return
        try:
            ser.write(build_nav_config_line(self._state.navConfig).encode("ascii"))
            self._last_navcfg_write_ts = time.time()
        except Exception:
            pass

    def snapshot(self) -> BridgeState:
        with self._lock:
            last = self._state.lastMessageAt
            is_stale = bool(
                self._state.connected
                and last is not None
                and (time.time() - last) > TELEMETRY_STALE_S
            )
            return BridgeState(
                connected=self._state.connected,
                mockEnabled=self._state.mockEnabled,
                serialPort=self._state.serialPort,
                baudRate=self._state.baudRate,
                lastError=self._state.lastError,
                lastMessageAt=self._state.lastMessageAt,
                stale=is_stale,
                motorLeftAbs=self._state.motorLeftAbs,
                motorRightAbs=self._state.motorRightAbs,
                navConfig=NavConfig(**asdict(self._state.navConfig)),
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

                # Discard the boat's diagnostic ack packet (all four sensor
                # fields == 111). It only confirms the radio link is alive and
                # would otherwise crowd out real sensor readings.
                if values[0] == 111 and values[1] == 111 and values[2] == 111 and values[3] == 111:
                    self._state.lastError = ""
                    continue

                # Straight mapping: rear/radar echo is values[0], front is
                # values[1] (the earlier front/rear swap has been reverted).
                self._state.telemetry = Telemetry(
                    usRadar=self._smooth_reading("usRadar", filter_near_reading(values[0])),
                    usFront=self._smooth_reading("usFront", filter_near_reading(values[1])),
                    usLeft=self._smooth_reading("usLeft", filter_near_reading(values[2])),
                    usRight=self._smooth_reading("usRight", filter_near_reading(values[3])),
                    radarAngle=values[4],
                    boatX=self._sim_boat_x,
                    boatY=self._sim_boat_y,
                )
                self._state.lastError = ""
                self._publish()

    def _smooth_reading(self, channel: str, value: int) -> int:
        """Median-filter a live sensor channel to reject single-frame spikes.

        Only in-range samples feed the median; if the whole window is
        out-of-range the sentinel is returned unchanged. Keeps at most
        SENSOR_MEDIAN_WINDOW recent samples per channel. Called under the lock
        from the reader loop.
        """
        hist = self._sensor_hist.get(channel)
        if hist is None:
            return value
        hist.append(value)
        if len(hist) > SENSOR_MEDIAN_WINDOW:
            del hist[0]
        valid = sorted(v for v in hist if v < OUT_OF_RANGE_CM)
        if not valid:
            return value
        return valid[len(valid) // 2]

    def _heartbeat_loop(self) -> None:
        """Periodically re-send the last command so the boat's failsafe stays armed.

        The Arduino stops its motors if no valid packet arrives within its
        failsafe window. Without a heartbeat, a boat that is told "go forward"
        and then left alone would be halted by the watchdog. Re-transmitting the
        current command keeps the RF link alive so the boat keeps doing exactly
        what the UI last asked, while a genuine link loss still trips the failsafe.
        """
        while True:
            time.sleep(HEARTBEAT_INTERVAL_S / 2.0)
            with self._lock:
                ser = self._serial
                if ser is None or not ser.is_open:
                    continue
                # Periodic nav-config resend: cheap keep-alive so a boat that
                # reboots (or is armed after the params were set) re-syncs its
                # tuning without a reflash. Rare enough not to load the link.
                if time.time() - self._last_navcfg_write_ts >= NAVCFG_RESEND_INTERVAL_S:
                    self._send_nav_config_locked()
                # Only retransmit if the UI hasn't sent a command recently. When
                # the UI is actively driving (commands every ~250ms) this never
                # fires, so the shore isn't flooded and the boat's return
                # telemetry gets a clear window. It only kicks in as a failsafe
                # keep-alive when the UI goes idle.
                if time.time() - self._last_serial_write_ts < HEARTBEAT_INTERVAL_S:
                    continue
                command = self._state.command
                # Keep-alive radar sweep: while the boat is stopped its servo
                # would sit still and telemetry would dry up (pings only stream
                # during motion). Advance radarAngle each idle beat so the servo
                # keeps scanning and fresh readings keep arriving even at rest.
                if command.leftSpeed == 0 and command.rightSpeed == 0:
                    self._idle_sweep_i = (self._idle_sweep_i + 1) % len(IDLE_SWEEP_ANGLES)
                    command = CommandState(
                        leftSpeed=0, rightSpeed=0, winchSpeed=command.winchSpeed,
                        radarAngle=IDLE_SWEEP_ANGLES[self._idle_sweep_i],
                        mode=command.mode,
                    )
                    self._state.command = command
                line = build_serial_line(
                    command, self._state.motorLeftAbs, self._state.motorRightAbs
                )
                try:
                    ser.write(line.encode("ascii"))
                    self._last_serial_write_ts = time.time()
                except Exception:
                    pass

    def _watchdog_loop(self) -> None:
        """Revive the telemetry stream when it goes stale on the real link.

        The radio bridge can hang (observed: lastMessageAt frozen for many
        seconds while still 'connected'). Reopening the serial port toggles DTR
        and resets the bridge, restoring the stream. We do this at most once per
        RECONNECT_COOLDOWN_S so a genuinely dead link isn't thrashed on every
        tick (repeated fast reconnects never let the boat finish booting).
        """
        while True:
            time.sleep(1.0)
            with self._lock:
                connected = self._state.connected
                ser = self._serial
                last = self._state.lastMessageAt
                port = self._last_port
                baud = self._last_baud
                since_reconnect = time.time() - self._last_reconnect_ts
            if not connected or ser is None or last is None or not port:
                continue
            if (time.time() - last) <= TELEMETRY_STALE_S:
                continue
            if since_reconnect < RECONNECT_COOLDOWN_S:
                continue
            with self._lock:
                self._state.lastError = "watchdog: telemetry stale \u2014 reopening port"
            try:
                self.connect(port, baud)
            except Exception as exc:
                with self._lock:
                    self._state.lastError = f"watchdog reconnect failed: {exc}"
                    self._last_reconnect_ts = time.time()

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
                
                # Update boat dynamics from the motor commands, using the EXACT
                # model the real boat obeys so the sim maneuvers like the water.
                # Each drive motor is only three-state on the boat: full ahead,
                # full astern, or stopped. build_serial_line forces a running
                # motor's magnitude to its per-motor calibration speed, and that
                # calibration exists ONLY to equalise the two motors' effective
                # thrust — so here both running motors deliver the SAME unit
                # thrust and the joystick decides only each motor's direction.
                # That is what produces the real maneuvering:
                #   both ahead               -> straight
                #   one ahead, other stopped -> forward arc (turns while moving)
                #   one ahead, other astern  -> spin in place
                # (The autonomous controller drives through this exact physics.)
                left_dir = motor_direction(self._state.command.leftSpeed)
                right_dir = motor_direction(self._state.command.rightSpeed)
                # Shared effective drive level (fraction of full 0..255 scale).
                # The two calibration speeds are tuned to be equal in effect, so
                # use their mean as the common thrust magnitude for both motors.
                motor_level = (
                    self._state.motorLeftAbs + self._state.motorRightAbs
                ) / 2.0 / 255.0
                # Throttle: net forward drive (both motors contribute equally).
                throttle = (left_dir + right_dir) / 2.0 * motor_level
                # Turn: differential drive (rotation) from the two directions.
                turn = (right_dir - left_dir) / 2.0 * motor_level

                # Linear momentum: thrust from the motors fights water drag, so
                # the hull glides on after the throttle drops instead of stopping
                # dead. Steady state (thrust == drag) gives throttle * max speed,
                # preserving the 45 cm/s cap while adding a ~0.7 s coast.
                thrust = throttle * MOCK_MAX_SPEED_CMS * MOCK_LINEAR_DRAG
                self._sim_boat_speed_cms += (thrust - MOCK_LINEAR_DRAG * self._sim_boat_speed_cms) * dt

                # Angular momentum: the yaw rate eases toward the commanded rate
                # (and keeps spinning briefly after the command changes) rather
                # than snapping instantly, so turns overshoot like a real hull.
                target_turn_rate = turn * MOCK_TURN_RATE
                self._sim_boat_turn_rate += (target_turn_rate - self._sim_boat_turn_rate) * MOCK_TURN_DRAG * dt
                self._sim_boat_heading_rad += self._sim_boat_turn_rate * dt
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
                #     echoed back; at servo 60° the front sensor points at the BOW
                #     (the fixed servo-to-bow offset, MOCK_BOW_OFFSET_DEG), just
                #     like the real hull — so the navigator's (base+servo-60)
                #     bearing recovery is geometrically faithful here too.
                max_range = 450
                fov_deg = 15
                boat_heading_deg = math.degrees(self._sim_boat_heading_rad)

                # The servo follows the last PC command (the firmware just echoes
                # cmd.radarAngle). No sweep is invented here; the PC drives it.
                sweep = self._state.command.radarAngle
                # Cast angle: subtract the bow offset so servo 60° => the front
                # sensor's ray points along the bow (heading), matching the real
                # geometry. Telemetry still echoes the RAW servo angle (sweep) —
                # the navigator applies the bow offset itself (base+servo-60).
                cast = sweep - MOCK_BOW_OFFSET_DEG

                us_front = self._cast_sensor_ray(boat_heading_deg + cast + 0, fov_deg, max_range)
                us_right = self._cast_sensor_ray(boat_heading_deg + cast + 90, fov_deg, max_range)
                us_back = self._cast_sensor_ray(boat_heading_deg + cast + 180, fov_deg, max_range)
                us_left = self._cast_sensor_ray(boat_heading_deg + cast + 270, fov_deg, max_range)

                # Real ultrasonic hardware is noisy: an occasional ping returns no
                # echo at all, another comes back as a phantom short reading, and
                # every good ping jitters a few cm. Inject that here so the nav
                # stack has to cope with dirty data exactly like on the water.
                us_front = self._noisy_reading(us_front, random, max_range)
                us_right = self._noisy_reading(us_right, random, max_range)
                us_back = self._noisy_reading(us_back, random, max_range)
                us_left = self._noisy_reading(us_left, random, max_range)

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

    def _noisy_reading(self, value: int, rng, max_range: int) -> int:
        """Corrupt one clean sensor reading the way real hardware would.

        Models three failure modes seen on cheap ultrasonic rangefinders:
          * dropout  — no echo returns, the driver reports out-of-range (999);
          * spike    — cross-talk / a passing wave fakes a short reading;
          * jitter   — every genuine echo wobbles a couple of cm.
        """
        roll = rng.random()
        if roll < MOCK_SENSOR_DROPOUT_P:
            return OUT_OF_RANGE_CM
        if roll < MOCK_SENSOR_DROPOUT_P + MOCK_SENSOR_SPIKE_P:
            return int(rng.uniform(MIN_VALID_DISTANCE_CM * 2, 80))
        if value >= OUT_OF_RANGE_CM:
            return OUT_OF_RANGE_CM
        noisy = value + rng.gauss(0.0, MOCK_SENSOR_JITTER_CM)
        return int(round(max(MIN_VALID_DISTANCE_CM * 2, min(max_range, noisy))))

    def _make_wall(self, x1: float, y1: float, x2: float, y2: float) -> dict:
        """A thin straight-line obstacle plus a cached row of sample points.

        Obstacles in the real pool are straight baffle walls, not blobs. We keep
        the segment endpoints for drawing/serialisation and pre-sample the
        centreline into overlapping points (each a disc of radius thickness/2)
        so the existing point-based ray caster sees one continuous 5 cm barrier.
        """
        thickness = MOCK_WALL_THICKNESS_CM
        radius = thickness / 2.0
        length = math.hypot(x2 - x1, y2 - y1)
        step = 3.0  # < 2*radius so the discs overlap into a solid line
        n = max(1, int(round(length / step)))
        pts = []
        for i in range(n + 1):
            t = i / n
            pts.append({
                "x": x1 + (x2 - x1) * t,
                "y": y1 + (y2 - y1) * t,
                "radius": radius,
            })
        return {
            "x1": float(x1), "y1": float(y1),
            "x2": float(x2), "y2": float(y2),
            "thickness": thickness,
            "_pts": pts,
        }

    def _create_rectangle_targets(self) -> list[dict]:
        """Three baffle walls inside the pool — EXACT real-world geometry.

        Pool: 2.2 m wide (x, half=110) × 4.5 m long (y, half=225). Each baffle is
        a single straight wall 90 cm long × 5 cm thick. TWO grow from the LEFT
        wall (x=-hx), positioned 1.5 m from the bottom and top end walls
        (y = -hy+150 = -75 and y = +hy-150 = +75); ONE grows from the RIGHT wall
        (x=+hx) exactly at the middle (y=0). The left tips reach x=-hx+90=-20 and
        the right tip reaches x=+hx-90=+20, so each leaves a 130 cm gap on the
        opposite side. Crossing bottom->top forces a weave: right -> left -> right.

          left  @ y=-75 (x:-110..-20) -> gap on the RIGHT
          right @ y=0    (x:+110..+20) -> gap on the LEFT
          left  @ y=+75 (x:-110..-20) -> gap on the RIGHT
        """
        hx = self._sim_bounds_half_x
        hy = self._sim_bounds_half_y
        self._sim_walls = [
            self._make_wall(-hx, -(hy - 150.0), -hx + 90.0, -(hy - 150.0)),  # left, 1.5m from bottom
            self._make_wall(hx, 0.0, hx - 90.0, 0.0),                        # right, middle
            self._make_wall(-hx, hy - 150.0, -hx + 90.0, hy - 150.0),        # left, 1.5m from top
        ]
        return []  # geometry lives in self._sim_walls, not as circle bodies


    def _create_serpentine_targets(self) -> list[dict]:
        """Three baffle walls forming a boustrophedon course (see the sketch).

        Each wall is a single thin straight segment. The gaps alternate side to
        side, so the only way from the bottom-left start to the top-right goal is
        to weave: right -> up -> left -> up -> right -> up.
        """
        hx = self._sim_bounds_half_x
        self._sim_walls = [
            self._make_wall(-hx, -225.0, 230.0, -225.0),  # lower divider: gap RIGHT
            self._make_wall(hx, 0.0, -230.0, 0.0),        # middle divider: gap LEFT
            self._make_wall(-hx, 225.0, 230.0, 225.0),    # upper divider: gap RIGHT
        ]
        return []  # geometry lives in self._sim_walls, not as circle bodies


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

        # Circle bodies (random scenario) plus the sampled points that make up
        # every thin straight-line baffle wall. Both are treated as discs: the
        # ray echoes off the nearest surface anywhere inside the beam cone.
        point_sources = list(self._sim_targets)
        for wall in self._sim_walls:
            point_sources.extend(wall["_pts"])

        for target in point_sources:
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

        if self.path == "/api/logs":
            self._send_json({"logs": list_navlogs()})
            return

        if self.path.startswith("/api/logs/"):
            self._serve_navlog(self.path[len("/api/logs/"):])
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
                scenario = str(body.get("scenario", "rectangle"))
                self._send_json({"state": asdict(BRIDGE.set_mock_enabled(enabled, scenario))})
                return

            if self.path == "/api/command":
                self._send_json({"state": asdict(BRIDGE.send_command(body))})
                return

            if self.path == "/api/motorconfig":
                self._send_json({"state": asdict(BRIDGE.set_motor_config(body))})
                return

            if self.path == "/api/navconfig":
                self._send_json({"state": asdict(BRIDGE.set_nav_config(body))})
                return

            if self.path == "/api/navlog":
                records = body.get("records", []) or []
                fname = navlog_append(records, reset=bool(body.get("reset")))
                self._send_json({"ok": True, "count": len(records), "file": fname})
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

    def _serve_navlog(self, name: str) -> None:
        """Stream one recorded run log (NDJSON) back for playback.

        Path is strictly validated to a `nav-*.ndjson` file inside LOG_DIR so a
        crafted name can't traverse out of the logs directory.
        """
        name = unquote(name).split("?", 1)[0]
        if ("/" in name or "\\" in name or ".." in name
                or not name.startswith("nav-") or not name.endswith(".ndjson")):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        file_path = (LOG_DIR / name).resolve()
        if not str(file_path).startswith(str(LOG_DIR.resolve())) or not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        raw = file_path.read_bytes()
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
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
            # Local single-user control UI: always serve the latest file so a
            # firmware/UI fix reaches the browser (and phone) without a manual
            # hard-refresh. Files are tiny, so skipping the cache costs nothing.
            self.send_header("Cache-Control", "no-store")
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