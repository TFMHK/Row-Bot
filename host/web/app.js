const connectBtn = document.getElementById("connectBtn");
const connectionState = document.getElementById("connectionState");
const modeSwitch = document.getElementById("modeSwitch");
const pullNetBtn = document.getElementById("pullNetBtn");
const releaseNetBtn = document.getElementById("releaseNetBtn");

const leftSpeedValue = document.getElementById("leftSpeedValue");
const rightSpeedValue = document.getElementById("rightSpeedValue");
const radarAngleValue = document.getElementById("radarAngleValue");
const radarDistanceValue = document.getElementById("radarDistanceValue");

const frontValue = document.getElementById("frontValue");
const leftValue = document.getElementById("leftValue");
const rightValue = document.getElementById("rightValue");
const radarValue = document.getElementById("radarValue");

const joystickBase = document.getElementById("joystickBase");
const joystickStick = document.getElementById("joystickStick");

const radarCanvas = document.getElementById("radarCanvas");
const radarCtx = radarCanvas.getContext("2d");

const state = {
  port: null,
  reader: null,
  writer: null,
  keepReading: false,
  commandLoopId: null,
  lastRadarAngleSent: 90,
  manualMode: true,
  telemetry: {
    usRadar: null,
    usFront: null,
    usLeft: null,
    usRight: null,
    radarAngle: 90,
  },
  cmd: {
    leftSpeed: 0,
    rightSpeed: 0,
    winchSpeed: 0,
    radarAngle: 90,
  },
  joystick: {
    x: 0,
    y: 0,
    active: false,
  },
  sweepDirection: 1,
  radarPoints: [],
};

const CONTROL_INTERVAL_MS = 80;
const AUTONOMOUS_SPEED = 150;
const AVOID_SPEED = 170;
const SAFE_DISTANCE_CM = 60;

connectBtn.addEventListener("click", onConnectClick);
modeSwitch.addEventListener("change", onModeChange);

setupJoystick();
setupWinchButtons();
requestAnimationFrame(drawRadar);

async function onConnectClick() {
  if (!("serial" in navigator)) {
    alert("הדפדפן לא תומך ב-Web Serial. מומלץ Chrome או Edge.");
    return;
  }

  if (state.port) {
    await disconnectSerial();
    return;
  }

  try {
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });

    state.port = port;
    state.writer = port.writable.getWriter();
    state.keepReading = true;

    setConnectedUI(true);
    startCommandLoop();
    readSerialLoop();
  } catch (err) {
    console.error(err);
    alert("נכשל חיבור ל-Serial.");
  }
}

async function disconnectSerial() {
  state.keepReading = false;

  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
    state.commandLoopId = null;
  }

  try {
    if (state.reader) {
      await state.reader.cancel();
      state.reader.releaseLock();
      state.reader = null;
    }
  } catch (err) {
    console.warn("reader close error", err);
  }

  try {
    if (state.writer) {
      await state.writer.write(new TextEncoder().encode("0,0,0,90\\n"));
      state.writer.releaseLock();
      state.writer = null;
    }
  } catch (err) {
    console.warn("writer close error", err);
  }

  try {
    if (state.port) {
      await state.port.close();
      state.port = null;
    }
  } catch (err) {
    console.warn("port close error", err);
  }

  setConnectedUI(false);
}

function setConnectedUI(connected) {
  connectBtn.textContent = connected ? "נתק" : "התחבר ל-Serial";
  connectionState.textContent = connected ? "מחובר" : "לא מחובר";
  connectionState.classList.toggle("connected", connected);
  connectionState.classList.toggle("disconnected", !connected);
}

function startCommandLoop() {
  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
  }

  state.commandLoopId = setInterval(async () => {
    if (!state.port || !state.writer) {
      return;
    }

    if (!state.manualMode) {
      updateAutonomousCommand();
    }

    state.lastRadarAngleSent = state.cmd.radarAngle;
    updateCommandUI();

    const line = `${state.cmd.leftSpeed},${state.cmd.rightSpeed},${state.cmd.winchSpeed},${state.cmd.radarAngle}\n`;
    try {
      await state.writer.write(new TextEncoder().encode(line));
    } catch (err) {
      console.error("send command failed", err);
    }
  }, CONTROL_INTERVAL_MS);
}

async function readSerialLoop() {
  if (!state.port?.readable) {
    return;
  }

  const decoder = new TextDecoder();
  let pending = "";

  while (state.port && state.keepReading) {
    try {
      state.reader = state.port.readable.getReader();

      while (state.keepReading) {
        const { value, done } = await state.reader.read();
        if (done) {
          break;
        }

        pending += decoder.decode(value, { stream: true });
        const lines = pending.split("\n");
        pending = lines.pop() ?? "";

        for (const rawLine of lines) {
          parseTelemetryLine(rawLine.trim());
        }
      }
    } catch (err) {
      console.error("read error", err);
      break;
    } finally {
      if (state.reader) {
        state.reader.releaseLock();
        state.reader = null;
      }
    }
  }

  if (state.port) {
    await disconnectSerial();
  }
}

function parseTelemetryLine(line) {
  if (!line) {
    return;
  }

  const parts = line.split(",").map((v) => Number.parseInt(v, 10));
  if (parts.length !== 5 || parts.some(Number.isNaN)) {
    console.warn("non-telemetry message:", line);
    return;
  }

  state.telemetry.usRadar = parts[0];
  state.telemetry.usFront = parts[1];
  state.telemetry.usLeft = parts[2];
  state.telemetry.usRight = parts[3];
  state.telemetry.radarAngle = parts[4];
  state.lastRadarAngleSent = parts[4];

  radarValue.textContent = String(parts[0]);
  frontValue.textContent = String(parts[1]);
  leftValue.textContent = String(parts[2]);
  rightValue.textContent = String(parts[3]);
  radarAngleValue.textContent = String(parts[4]);

  radarDistanceValue.textContent = parts[0] === 999 ? "OUT" : String(parts[0]);
  addRadarPoint(parts[4], parts[0]);
}

function onModeChange() {
  state.manualMode = !modeSwitch.checked;
  resetJoystick();

  if (!state.manualMode) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
  }

  updateCommandUI();
}

function setupJoystick() {
  const radius = () => joystickBase.clientWidth / 2;

  const onPointerMove = (event) => {
    if (!state.joystick.active || !state.manualMode) {
      return;
    }

    const rect = joystickBase.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const dx = event.clientX - centerX;
    const dy = event.clientY - centerY;

    const max = radius() - joystickStick.clientWidth / 2;
    const len = Math.hypot(dx, dy);
    const scale = len > max ? max / len : 1;

    const x = dx * scale;
    const y = dy * scale;

    state.joystick.x = x / max;
    state.joystick.y = y / max;

    joystickStick.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
    updateManualCommandFromJoystick();
  };

  joystickBase.addEventListener("pointerdown", (event) => {
    if (!state.manualMode) {
      return;
    }
    state.joystick.active = true;
    joystickBase.setPointerCapture(event.pointerId);
    onPointerMove(event);
  });

  joystickBase.addEventListener("pointermove", onPointerMove);

  const stop = (event) => {
    if (state.joystick.active) {
      state.joystick.active = false;
      if (event?.pointerId !== undefined) {
        joystickBase.releasePointerCapture(event.pointerId);
      }
      resetJoystick();
    }
  };

  joystickBase.addEventListener("pointerup", stop);
  joystickBase.addEventListener("pointercancel", stop);
}

function updateManualCommandFromJoystick() {
  const forward = -state.joystick.y;
  const turn = state.joystick.x;

  const left = clamp(Math.round((forward + turn) * 255), -255, 255);
  const right = clamp(Math.round((forward - turn) * 255), -255, 255);

  state.cmd.leftSpeed = left;
  state.cmd.rightSpeed = right;

  updateCommandUI();
}

function resetJoystick() {
  state.joystick.x = 0;
  state.joystick.y = 0;
  joystickStick.style.transform = "translate(-50%, -50%)";

  if (state.manualMode) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
  }

  updateCommandUI();
}

function setupWinchButtons() {
  const bindHold = (button, speed) => {
    const start = () => {
      state.cmd.winchSpeed = speed;
    };
    const stop = () => {
      state.cmd.winchSpeed = 0;
    };

    button.addEventListener("pointerdown", start);
    button.addEventListener("pointerup", stop);
    button.addEventListener("pointerleave", stop);
    button.addEventListener("pointercancel", stop);
  };

  bindHold(pullNetBtn, 200);
  bindHold(releaseNetBtn, -200);
}

function updateAutonomousCommand() {
  const usFront = state.telemetry.usFront ?? 999;
  const usLeft = state.telemetry.usLeft ?? 999;
  const usRight = state.telemetry.usRight ?? 999;

  const obstacleAhead = usFront !== 999 && usFront < SAFE_DISTANCE_CM;

  if (obstacleAhead) {
    if (usLeft > usRight) {
      state.cmd.leftSpeed = -AVOID_SPEED;
      state.cmd.rightSpeed = AVOID_SPEED;
    } else {
      state.cmd.leftSpeed = AVOID_SPEED;
      state.cmd.rightSpeed = -AVOID_SPEED;
    }
  } else {
    state.cmd.leftSpeed = AUTONOMOUS_SPEED;
    state.cmd.rightSpeed = AUTONOMOUS_SPEED;
  }

  let nextAngle = state.cmd.radarAngle + state.sweepDirection * 4;
  if (nextAngle >= 180) {
    nextAngle = 180;
    state.sweepDirection = -1;
  }
  if (nextAngle <= 0) {
    nextAngle = 0;
    state.sweepDirection = 1;
  }
  state.cmd.radarAngle = nextAngle;
}

function updateCommandUI() {
  leftSpeedValue.textContent = String(state.cmd.leftSpeed);
  rightSpeedValue.textContent = String(state.cmd.rightSpeed);
  radarAngleValue.textContent = String(state.cmd.radarAngle);
}

function addRadarPoint(angleDeg, distanceCm) {
  const normalizedDistance = distanceCm === 999 ? 340 : clamp(distanceCm, 0, 340);

  state.radarPoints.push({
    angleDeg,
    distanceCm: normalizedDistance,
    ts: Date.now(),
  });

  const cutoff = Date.now() - 7000;
  state.radarPoints = state.radarPoints.filter((p) => p.ts > cutoff);
}

function drawRadar() {
  const w = radarCanvas.width;
  const h = radarCanvas.height;
  const centerX = w / 2;
  const centerY = h - 24;
  const maxRadius = Math.min(w * 0.45, h - 30);

  radarCtx.clearRect(0, 0, w, h);

  radarCtx.fillStyle = "rgba(0, 20, 16, 0.85)";
  radarCtx.fillRect(0, 0, w, h);

  radarCtx.strokeStyle = "rgba(0, 255, 180, 0.28)";
  radarCtx.lineWidth = 1;

  for (let i = 1; i <= 4; i += 1) {
    radarCtx.beginPath();
    radarCtx.arc(centerX, centerY, (maxRadius / 4) * i, Math.PI, 2 * Math.PI);
    radarCtx.stroke();
  }

  for (let angle = 0; angle <= 180; angle += 30) {
    const rad = toRadarRad(angle);
    const x = centerX + Math.cos(rad) * maxRadius;
    const y = centerY + Math.sin(rad) * maxRadius;
    radarCtx.beginPath();
    radarCtx.moveTo(centerX, centerY);
    radarCtx.lineTo(x, y);
    radarCtx.stroke();
  }

  const now = Date.now();
  for (const point of state.radarPoints) {
    const alpha = Math.max(0.1, 1 - (now - point.ts) / 7000);
    const rad = toRadarRad(point.angleDeg);
    const r = (point.distanceCm / 340) * maxRadius;
    const x = centerX + Math.cos(rad) * r;
    const y = centerY + Math.sin(rad) * r;

    radarCtx.fillStyle = `rgba(0, 255, 163, ${alpha})`;
    radarCtx.beginPath();
    radarCtx.arc(x, y, 3, 0, Math.PI * 2);
    radarCtx.fill();
  }

  const sweepRad = toRadarRad(state.lastRadarAngleSent);
  radarCtx.strokeStyle = "rgba(44, 255, 197, 0.85)";
  radarCtx.lineWidth = 2;
  radarCtx.beginPath();
  radarCtx.moveTo(centerX, centerY);
  radarCtx.lineTo(centerX + Math.cos(sweepRad) * maxRadius, centerY + Math.sin(sweepRad) * maxRadius);
  radarCtx.stroke();

  requestAnimationFrame(drawRadar);
}

function toRadarRad(angleDeg) {
  return Math.PI - (angleDeg * Math.PI) / 180;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}
