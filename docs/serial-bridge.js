// serial-bridge.js — client-side port of the Python SerialBridge.
//
// When the UI is served as a STATIC site (GitHub Pages / opened directly) there
// is no control_server.py. This module reproduces every server responsibility
// in the browser and talks to the shore Arduino directly over the Web Serial
// API (USB-OTG on Android Chrome). app.js routes its /api/* calls here whenever
// SERVERLESS mode is active, so the whole navigation stack runs unchanged.
//
// Responsibilities mirrored from control_server.py:
//   * open/read/write the USB serial port (replaces pyserial)
//   * heartbeat re-transmit so the boat's failsafe stays armed
//   * watchdog reopen when telemetry goes stale
//   * per-channel median spike rejection
//   * host-side motor shaping + channel swap (build_serial_line)
//   * mock world simulator (physics + ultrasonic ray casting)
//   * state fan-out to subscribers (replaces the SSE stream)
(function () {
  "use strict";

  // --- Constants (kept in lock-step with control_server.py) ------------------
  const HEARTBEAT_INTERVAL_S = 0.4;
  const TELEMETRY_STALE_S = 3.0;
  const SERIAL_MIN_WRITE_INTERVAL_S = 0.1;
  const RECONNECT_COOLDOWN_S = 6.0;
  const SENSOR_MEDIAN_WINDOW = 3;
  const IDLE_SWEEP_ANGLES = [0, 30, 60, 90, 120, 150, 180];

  const MOTOR_DEADZONE = 35;
  const MOTOR_LOW_SPEED = 70;
  const MOTOR_HIGH_SPEED = 100;
  const MOTOR_LEFT_ABS_DEFAULT = 90;
  const MOTOR_RIGHT_ABS_DEFAULT = 80;

  const MIN_VALID_DISTANCE_CM = 10;
  const OUT_OF_RANGE_CM = 999;

  const MOCK_MAX_SPEED_CMS = 45.0;
  const MOCK_TURN_RATE = Math.PI * 0.25;
  const MOCK_LINEAR_DRAG = 1.4;
  const MOCK_TURN_DRAG = 2.5;
  const MOCK_SENSOR_DROPOUT_P = 0.03;
  const MOCK_SENSOR_SPIKE_P = 0.03;
  const MOCK_SENSOR_JITTER_CM = 2.0;
  const MOCK_WALL_THICKNESS_CM = 5.0;

  const BAUD_RATE = 115200;

  // --- Pure helpers ----------------------------------------------------------
  function clamp(v, lo, hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  function filterNearReading(value) {
    return value >= MIN_VALID_DISTANCE_CM ? value : OUT_OF_RANGE_CM;
  }

  function shapeMotorSpeed(value) {
    const magnitude = Math.abs(value);
    if (magnitude < MOTOR_DEADZONE) return 0;
    const stepped = Math.min(Math.max(magnitude, MOTOR_LOW_SPEED), MOTOR_HIGH_SPEED);
    return value > 0 ? stepped : -stepped;
  }

  // Exact port of build_serial_line(): swap L/R channels, apply per-motor
  // absolute speed, invert winch polarity, append the mode field.
  function buildSerialLine(command, motorLeftAbs, motorRightAbs) {
    let physicalLeft = shapeMotorSpeed(command.rightSpeed);
    let physicalRight = shapeMotorSpeed(command.leftSpeed);
    // The abs multipliers follow the SAME channel swap as above, so
    // motorLeftAbs governs the physical left motor (emitted on the right slot)
    // and motorRightAbs the physical right motor.
    if (physicalLeft !== 0) {
      const mag = clamp(Math.trunc(motorRightAbs), 0, MOTOR_HIGH_SPEED);
      physicalLeft = physicalLeft > 0 ? mag : -mag;
    }
    if (physicalRight !== 0) {
      const mag = clamp(Math.trunc(motorLeftAbs), 0, MOTOR_HIGH_SPEED);
      physicalRight = physicalRight > 0 ? mag : -mag;
    }
    const physicalWinch = -command.winchSpeed;
    return `${physicalLeft},${physicalRight},${physicalWinch},${command.radarAngle},${command.mode}\n`;
  }

  function gauss(mean, sd) {
    let u = 0;
    let v = 0;
    while (u === 0) u = Math.random();
    while (v === 0) v = Math.random();
    return mean + sd * Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
  }

  function newCommand() {
    return { leftSpeed: 0, rightSpeed: 0, winchSpeed: 0, radarAngle: 90, mode: 0 };
  }

  function newTelemetry() {
    return {
      usRadar: null, usFront: null, usLeft: null, usRight: null,
      radarAngle: 90, boatHeadingDeg: 0, boatX: 0, boatY: 0,
    };
  }

  class LocalBridge {
    constructor() {
      this._state = {
        connected: false,
        mockEnabled: false,
        serialPort: "",
        baudRate: BAUD_RATE,
        lastError: "",
        lastMessageAt: null,
        motorLeftAbs: MOTOR_LEFT_ABS_DEFAULT,
        motorRightAbs: MOTOR_RIGHT_ABS_DEFAULT,
        telemetry: newTelemetry(),
        command: newCommand(),
      };

      // --- Serial handles ---
      this._port = null;
      this._reader = null;
      this._writer = null;
      this._lastSerialWriteTs = 0;
      this._lastReconnectTs = 0;
      this._sensorHist = { usRadar: [], usFront: [], usLeft: [], usRight: [] };
      this._idleSweepI = 0;

      // --- Subscribers (SSE replacement) ---
      this._subscribers = new Set();

      // --- Navigation log (NDJSON kept in memory, downloadable) ---
      this._navRecords = [];
      this._navRunName = "";

      // --- Mock simulator state ---
      this._simBoatX = 0;
      this._simBoatY = 0;
      this._simHeadingRad = 0;
      this._simSpeedCms = 0;
      this._simTurnRate = 0;
      this._simTargets = [];
      this._simWalls = [];
      this._simLastTs = performance.now() / 1000;
      this._simBoundsHalfX = 110.0;
      this._simBoundsHalfY = 225.0;
      this._simScenario = "rectangle";
      this._simStart = null;
      this._simGoal = null;

      // Background loops (mirror the server's daemon threads).
      setInterval(() => this._mockTick(), 50);
      setInterval(() => this._heartbeatTick(), (HEARTBEAT_INTERVAL_S / 2) * 1000);
      setInterval(() => this._watchdogTick(), 1000);
    }

    // --- Public HTTP-like dispatch (used by app.js getJson/postJson) ---------
    async request(method, path, body) {
      if (method === "GET") {
        if (path === "/api/state") return { state: this.snapshot() };
        if (path === "/api/ports") return { ports: await this.listPorts() };
        if (path === "/api/world") return this.worldSnapshot();
      } else {
        if (path === "/api/disconnect") return { state: this.disconnect() };
        if (path === "/api/mock") return { state: this.setMockEnabled(!!body.enabled, body.scenario || "rectangle") };
        if (path === "/api/motorconfig") return { state: this.setMotorConfig(body) };
        if (path === "/api/command") return { state: this.sendCommand(body) };
        if (path === "/api/navlog") return this.navlog(body);
      }
      throw new Error(`404 ${method} ${path}`);
    }

    // --- Subscriptions -------------------------------------------------------
    subscribe(cb) {
      this._subscribers.add(cb);
      cb(this.snapshot());
      return () => this._subscribers.delete(cb);
    }

    _publish() {
      const snap = this.snapshot();
      for (const cb of this._subscribers) {
        try { cb(snap); } catch (e) { /* ignore subscriber errors */ }
      }
    }

    // --- Snapshot ------------------------------------------------------------
    snapshot() {
      const last = this._state.lastMessageAt;
      const stale = !!(this._state.connected && last != null && (Date.now() / 1000 - last) > TELEMETRY_STALE_S);
      return {
        connected: this._state.connected,
        mockEnabled: this._state.mockEnabled,
        serialPort: this._state.serialPort,
        baudRate: this._state.baudRate,
        lastError: this._state.lastError,
        lastMessageAt: this._state.lastMessageAt,
        stale,
        motorLeftAbs: this._state.motorLeftAbs,
        motorRightAbs: this._state.motorRightAbs,
        telemetry: Object.assign({}, this._state.telemetry),
        command: Object.assign({}, this._state.command),
      };
    }

    // --- Ports ---------------------------------------------------------------
    async listPorts() {
      if (!("serial" in navigator)) return [];
      try {
        const ports = await navigator.serial.getPorts();
        return ports.map((_, i) => `USB #${i + 1}`);
      } catch (e) {
        return [];
      }
    }

    // --- Connect via the browser device chooser (requires a user gesture) ----
    async connectViaPrompt() {
      if (!("serial" in navigator)) {
        throw new Error("הדפדפן לא תומך ב-Web Serial (נדרש Chrome/Edge באנדרואיד)");
      }
      // requestPort MUST be called synchronously inside the click gesture — do
      // it first, before any await, so transient activation is still valid.
      const port = await navigator.serial.requestPort();
      await this._openPort(port);
      return this.snapshot();
    }

    async _openPort(port) {
      this._disconnectQuiet(false);
      this._state.mockEnabled = false;
      await port.open({ baudRate: BAUD_RATE });
      this._port = port;
      try {
        this._writer = port.writable.getWriter();
      } catch (e) {
        this._writer = null;
      }
      this._state.connected = true;
      this._state.serialPort = "USB";
      this._state.baudRate = BAUD_RATE;
      this._state.lastError = "";
      // Grace period: opening the port toggles DTR and reboots the Arduino, so
      // give it a fresh-data window before the watchdog can trip.
      this._state.lastMessageAt = Date.now() / 1000;
      this._lastReconnectTs = Date.now() / 1000;
      this._readLoop(port);
      this._publish();
    }

    disconnect() {
      this._disconnectQuiet(true);
      this._state.mockEnabled = false;
      this._publish();
      return this.snapshot();
    }

    _disconnectQuiet(sendStop) {
      const port = this._port;
      this._port = null; // stop the read loop from re-acquiring
      if (this._writer) {
        try {
          if (sendStop) this._writer.write(new TextEncoder().encode("0,0,0,90,0\n"));
        } catch (e) { /* ignore */ }
        try { this._writer.releaseLock(); } catch (e) { /* ignore */ }
        this._writer = null;
      }
      if (this._reader) {
        try { this._reader.cancel(); } catch (e) { /* ignore */ }
        this._reader = null;
      }
      if (port) {
        // close() after the read loop releases its lock; fire-and-forget.
        Promise.resolve().then(() => port.close().catch(() => {}));
      }
      this._state.connected = false;
      this._state.serialPort = "";
      this._state.command = newCommand();
    }

    async _readLoop(port) {
      let buf = "";
      const decoder = new TextDecoder();
      while (this._port === port && port.readable) {
        let reader;
        try {
          reader = port.readable.getReader();
        } catch (e) {
          break;
        }
        this._reader = reader;
        try {
          while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            if (!value) continue;
            buf += decoder.decode(value, { stream: true });
            let idx;
            while ((idx = buf.indexOf("\n")) >= 0) {
              const line = buf.slice(0, idx).trim();
              buf = buf.slice(idx + 1);
              if (line) this._handleLine(line);
            }
          }
        } catch (e) {
          this._state.lastError = `Serial read failed: ${e.message}`;
        } finally {
          try { reader.releaseLock(); } catch (e2) { /* ignore */ }
        }
        if (this._port !== port) break;
      }
    }

    _handleLine(line) {
      this._state.lastMessageAt = Date.now() / 1000;
      const parts = line.split(",");
      if (parts.length !== 5) {
        this._state.lastError = line;
        this._publish();
        return;
      }
      const values = parts.map((p) => parseInt(p, 10));
      if (values.some((n) => Number.isNaN(n))) {
        this._state.lastError = line;
        this._publish();
        return;
      }
      // Drop the boat's diagnostic ack packet (all four sensor fields == 111).
      if (values[0] === 111 && values[1] === 111 && values[2] === 111 && values[3] === 111) {
        this._state.lastError = "";
        return;
      }
      this._state.telemetry = {
        usRadar: this._smooth("usRadar", filterNearReading(values[0])),
        usFront: this._smooth("usFront", filterNearReading(values[1])),
        usLeft: this._smooth("usLeft", filterNearReading(values[2])),
        usRight: this._smooth("usRight", filterNearReading(values[3])),
        radarAngle: values[4],
        boatX: this._simBoatX,
        boatY: this._simBoatY,
        boatHeadingDeg: 0,
      };
      this._state.lastError = "";
      this._publish();
    }

    _smooth(channel, value) {
      const hist = this._sensorHist[channel];
      if (!hist) return value;
      hist.push(value);
      if (hist.length > SENSOR_MEDIAN_WINDOW) hist.shift();
      const valid = hist.filter((v) => v < OUT_OF_RANGE_CM).sort((a, b) => a - b);
      if (!valid.length) return value;
      return valid[Math.floor(valid.length / 2)];
    }

    async _write(str) {
      const now = Date.now() / 1000;
      if (this._writer) {
        try {
          await this._writer.write(new TextEncoder().encode(str));
          this._lastSerialWriteTs = now;
        } catch (e) {
          this._state.lastError = `Serial write failed: ${e.message}`;
        }
      }
    }

    // --- Commands ------------------------------------------------------------
    sendCommand(payload) {
      if (!this._port && !this._state.mockEnabled) {
        throw new Error("Serial port is not connected");
      }
      const command = {
        leftSpeed: clamp(Math.trunc(payload.leftSpeed || 0), -255, 255),
        rightSpeed: clamp(Math.trunc(payload.rightSpeed || 0), -255, 255),
        winchSpeed: clamp(Math.trunc(payload.winchSpeed || 0), -255, 255),
        radarAngle: clamp(Math.trunc(payload.radarAngle != null ? payload.radarAngle : 90), 0, 180),
        mode: clamp(Math.trunc(payload.mode != null ? payload.mode : this._state.command.mode), 0, 1),
      };
      const line = buildSerialLine(command, this._state.motorLeftAbs, this._state.motorRightAbs);
      const now = Date.now() / 1000;
      if (this._port && now - this._lastSerialWriteTs >= SERIAL_MIN_WRITE_INTERVAL_S) {
        this._write(line);
      }
      this._state.command = command;
      this._state.lastError = "";
      this._publish();
      return this.snapshot();
    }

    setMotorConfig(payload) {
      if (payload.motorLeftAbs != null) {
        this._state.motorLeftAbs = clamp(Math.trunc(payload.motorLeftAbs), 0, MOTOR_HIGH_SPEED);
      }
      if (payload.motorRightAbs != null) {
        this._state.motorRightAbs = clamp(Math.trunc(payload.motorRightAbs), 0, MOTOR_HIGH_SPEED);
      }
      this._publish();
      return this.snapshot();
    }

    _heartbeatTick() {
      if (!this._port) return;
      const now = Date.now() / 1000;
      if (now - this._lastSerialWriteTs < HEARTBEAT_INTERVAL_S) return;
      let command = this._state.command;
      if (command.leftSpeed === 0 && command.rightSpeed === 0) {
        this._idleSweepI = (this._idleSweepI + 1) % IDLE_SWEEP_ANGLES.length;
        command = {
          leftSpeed: 0, rightSpeed: 0, winchSpeed: command.winchSpeed,
          radarAngle: IDLE_SWEEP_ANGLES[this._idleSweepI], mode: command.mode,
        };
        this._state.command = command;
      }
      this._write(buildSerialLine(command, this._state.motorLeftAbs, this._state.motorRightAbs));
    }

    async _watchdogTick() {
      const now = Date.now() / 1000;
      const last = this._state.lastMessageAt;
      if (!this._state.connected || !this._port || last == null) return;
      if (now - last <= TELEMETRY_STALE_S) return;
      if (now - this._lastReconnectTs < RECONNECT_COOLDOWN_S) return;
      this._state.lastError = "watchdog: telemetry stale — reopening port";
      this._lastReconnectTs = now;
      const port = this._port;
      try {
        this._disconnectQuiet(false);
        await new Promise((r) => setTimeout(r, 300));
        await this._openPort(port);
      } catch (e) {
        this._state.lastError = `watchdog reconnect failed: ${e.message}`;
      }
    }

    // --- Navigation log (in-memory NDJSON, downloadable) ---------------------
    navlog(body) {
      if (body.reset || !this._navRunName) {
        const ts = new Date().toISOString().replace(/[:.]/g, "-");
        this._navRunName = `nav-${ts}.ndjson`;
        this._navRecords = [];
      }
      const recs = Array.isArray(body.records) ? body.records : [];
      for (const r of recs) this._navRecords.push(r);
      return { file: this._navRunName };
    }

    downloadNavLog() {
      const text = this._navRecords.map((r) => JSON.stringify(r)).join("\n") + "\n";
      const blob = new Blob([text], { type: "application/x-ndjson" });
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = this._navRunName || "nav-log.ndjson";
      a.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    }

    // --- Mock world ----------------------------------------------------------
    setMockEnabled(enabled, scenario) {
      if (enabled) {
        this._disconnectQuiet(true);
        this._state.mockEnabled = true;
        this._state.serialPort = "MOCK";
        this._state.lastError = "";
        this._state.lastMessageAt = Date.now() / 1000;
        this._state.command = newCommand();
        this._simScenario = scenario;
        this._simWalls = [];
        if (scenario === "serpentine") {
          this._createSerpentine();
          this._simBoatX = -(this._simBoundsHalfX - 80);
          this._simBoatY = -(this._simBoundsHalfY - 80);
          this._simHeadingRad = 0;
          this._simGoal = { x: this._simBoundsHalfX - 80, y: this._simBoundsHalfY - 80 };
        } else if (scenario === "random") {
          this._createRandom(24);
          this._simBoatX = 0;
          this._simBoatY = 0;
          this._simHeadingRad = 0;
          this._simGoal = null;
        } else {
          this._createRectangle();
          this._simBoatX = -(this._simBoundsHalfX - 20);
          this._simBoatY = -(this._simBoundsHalfY - 30);
          this._simHeadingRad = 0;
          this._simGoal = { x: -(this._simBoundsHalfX - 20), y: this._simBoundsHalfY - 30 };
        }
        this._simStart = { x: this._simBoatX, y: this._simBoatY };
        this._simSpeedCms = 0;
        this._simTurnRate = 0;
        this._simLastTs = performance.now() / 1000;
        this._state.telemetry = {
          usRadar: 180, usFront: 120, usLeft: 95, usRight: 105, radarAngle: 90,
          boatX: this._simBoatX, boatY: this._simBoatY,
          boatHeadingDeg: Math.round((this._simHeadingRad * 180) / Math.PI) % 360,
        };
      } else {
        this._state.mockEnabled = false;
        this._state.serialPort = "";
        this._state.command = newCommand();
      }
      this._publish();
      return this.snapshot();
    }

    worldSnapshot() {
      return {
        mockEnabled: this._state.mockEnabled,
        boundsHalfX: this._simBoundsHalfX,
        boundsHalfY: this._simBoundsHalfY,
        maxRange: 450,
        targets: this._simTargets.map((t) => Object.assign({}, t)),
        walls: this._simWalls.map((w) => ({
          x1: w.x1, y1: w.y1, x2: w.x2, y2: w.y2, thickness: w.thickness,
        })),
        scenario: this._simScenario,
        start: this._simStart ? Object.assign({}, this._simStart) : null,
        goal: this._simGoal ? Object.assign({}, this._simGoal) : null,
      };
    }

    _makeWall(x1, y1, x2, y2) {
      const thickness = MOCK_WALL_THICKNESS_CM;
      const radius = thickness / 2;
      const length = Math.hypot(x2 - x1, y2 - y1);
      const step = 3.0;
      const n = Math.max(1, Math.round(length / step));
      const pts = [];
      for (let i = 0; i <= n; i++) {
        const t = i / n;
        pts.push({ x: x1 + (x2 - x1) * t, y: y1 + (y2 - y1) * t, radius });
      }
      return { x1, y1, x2, y2, thickness, _pts: pts };
    }

    _createRectangle() {
      const hx = this._simBoundsHalfX;
      this._simWalls = [
        this._makeWall(-hx, -70.0, -35.0, -70.0),
        this._makeWall(hx, 0.0, 35.0, 0.0),
        this._makeWall(-hx, 70.0, -35.0, 70.0),
      ];
      this._simTargets = [];
    }

    _createSerpentine() {
      const hx = this._simBoundsHalfX;
      this._simWalls = [
        this._makeWall(-hx, -225.0, 230.0, -225.0),
        this._makeWall(hx, 0.0, -230.0, 0.0),
        this._makeWall(-hx, 225.0, 230.0, 225.0),
      ];
      this._simTargets = [];
    }

    _createRandom(count) {
      const margin = 40;
      const minX = -this._simBoundsHalfX + margin;
      const maxX = this._simBoundsHalfX - margin;
      const minY = -this._simBoundsHalfY + margin;
      const maxY = this._simBoundsHalfY - margin;
      const startClear = 110;
      const targets = [];
      for (let i = 0; i < count; i++) {
        let x = 0;
        let y = 0;
        let radius = 14;
        for (let a = 0; a < 50; a++) {
          x = minX + Math.random() * (maxX - minX);
          y = minY + Math.random() * (maxY - minY);
          radius = 14 + Math.random() * 26;
          if (Math.hypot(x, y) - radius >= startClear) break;
        }
        targets.push({ x, y, radius });
      }
      this._simTargets = targets;
      this._simWalls = [];
    }

    _mockTick() {
      if (!this._state.mockEnabled) return;
      const now = performance.now() / 1000;
      const dt = Math.max(0.01, now - this._simLastTs);
      this._simLastTs = now;

      const leftCmd = shapeMotorSpeed(this._state.command.leftSpeed);
      const rightCmd = shapeMotorSpeed(this._state.command.rightSpeed);
      const throttle = (leftCmd + rightCmd) / (2 * 255);
      const turn = (rightCmd - leftCmd) / (2 * 255);

      const thrust = throttle * MOCK_MAX_SPEED_CMS * MOCK_LINEAR_DRAG;
      this._simSpeedCms += (thrust - MOCK_LINEAR_DRAG * this._simSpeedCms) * dt;

      const targetTurnRate = turn * MOCK_TURN_RATE;
      this._simTurnRate += (targetTurnRate - this._simTurnRate) * MOCK_TURN_DRAG * dt;
      this._simHeadingRad += this._simTurnRate * dt;
      while (this._simHeadingRad < 0) this._simHeadingRad += 2 * Math.PI;
      while (this._simHeadingRad >= 2 * Math.PI) this._simHeadingRad -= 2 * Math.PI;

      this._simBoatX += Math.sin(this._simHeadingRad) * this._simSpeedCms * dt;
      this._simBoatY += Math.cos(this._simHeadingRad) * this._simSpeedCms * dt;
      this._simBoatX = clamp(this._simBoatX, -this._simBoundsHalfX, this._simBoundsHalfX);
      this._simBoatY = clamp(this._simBoatY, -this._simBoundsHalfY, this._simBoundsHalfY);

      const maxRange = 450;
      const fovDeg = 15;
      const headingDeg = (this._simHeadingRad * 180) / Math.PI;
      const sweep = this._state.command.radarAngle;

      let usFront = this._castRay(headingDeg + sweep + 0, fovDeg, maxRange);
      let usRight = this._castRay(headingDeg + sweep + 90, fovDeg, maxRange);
      let usBack = this._castRay(headingDeg + sweep + 180, fovDeg, maxRange);
      let usLeft = this._castRay(headingDeg + sweep + 270, fovDeg, maxRange);

      usFront = this._noisy(usFront, maxRange);
      usRight = this._noisy(usRight, maxRange);
      usBack = this._noisy(usBack, maxRange);
      usLeft = this._noisy(usLeft, maxRange);

      this._state.telemetry = {
        usRadar: usBack,
        usFront,
        usLeft,
        usRight,
        radarAngle: sweep,
        boatHeadingDeg: Math.trunc(headingDeg) % 360,
        boatX: this._simBoatX,
        boatY: this._simBoatY,
      };
      this._state.lastMessageAt = Date.now() / 1000;
      this._state.connected = false;
      this._publish();
    }

    _noisy(value, maxRange) {
      const roll = Math.random();
      if (roll < MOCK_SENSOR_DROPOUT_P) return OUT_OF_RANGE_CM;
      if (roll < MOCK_SENSOR_DROPOUT_P + MOCK_SENSOR_SPIKE_P) {
        return Math.trunc(MIN_VALID_DISTANCE_CM * 2 + Math.random() * (80 - MIN_VALID_DISTANCE_CM * 2));
      }
      if (value >= OUT_OF_RANGE_CM) return OUT_OF_RANGE_CM;
      const noisy = value + gauss(0, MOCK_SENSOR_JITTER_CM);
      return Math.round(Math.max(MIN_VALID_DISTANCE_CM * 2, Math.min(maxRange, noisy)));
    }

    _castWallDistance(centerRad, maxRange) {
      const sinT = Math.sin(centerRad);
      const cosT = Math.cos(centerRad);
      const halfX = this._simBoundsHalfX;
      const halfY = this._simBoundsHalfY;
      const bx = this._simBoatX;
      const by = this._simBoatY;
      let nearest = Infinity;
      const eps = 1e-6;
      if (Math.abs(sinT) > eps) {
        for (const wx of [-halfX, halfX]) {
          const t = (wx - bx) / sinT;
          if (t > 0) {
            const yHit = by + t * cosT;
            if (yHit >= -halfY - eps && yHit <= halfY + eps && t < nearest) nearest = t;
          }
        }
      }
      if (Math.abs(cosT) > eps) {
        for (const wy of [-halfY, halfY]) {
          const t = (wy - by) / cosT;
          if (t > 0) {
            const xHit = bx + t * sinT;
            if (xHit >= -halfX - eps && xHit <= halfX + eps && t < nearest) nearest = t;
          }
        }
      }
      return nearest;
    }

    _castRay(centerDeg, fovDeg, maxRange) {
      let nearest = maxRange + 1;
      const wallDist = this._castWallDistance((centerDeg * Math.PI) / 180, maxRange);
      if (wallDist <= maxRange) nearest = wallDist;

      const pointSources = this._simTargets.slice();
      for (const wall of this._simWalls) {
        for (const p of wall._pts) pointSources.push(p);
      }

      for (const target of pointSources) {
        const dx = target.x - this._simBoatX;
        const dy = target.y - this._simBoatY;
        const dist = Math.hypot(dx, dy);
        if (dist > maxRange + target.radius) continue;
        const targetDeg = (((Math.atan2(dx, dy) * 180) / Math.PI) % 360 + 360) % 360;
        const angleDiff = Math.abs(((centerDeg - targetDeg + 180) % 360 + 360) % 360 - 180);
        const angularRadius = dist > 1e-6
          ? (Math.asin(Math.min(1.0, target.radius / dist)) * 180) / Math.PI
          : 90.0;
        if (angleDiff - angularRadius > fovDeg / 2) continue;
        const impact = Math.max(20, dist - target.radius);
        if (impact < nearest) nearest = impact;
      }
      return nearest <= maxRange ? Math.round(nearest) : 999;
    }
  }

  window.LocalBridge = new LocalBridge();
})();
