const connectBtn = document.getElementById("connectBtn");
const connectionState = document.getElementById("connectionState");
const modeSwitch = document.getElementById("modeSwitch");
const pullNetBtn = document.getElementById("pullNetBtn");
const releaseNetBtn = document.getElementById("releaseNetBtn");
const mockSwitch = document.getElementById("mockSwitch");
const portSelect = document.getElementById("portSelect");
const refreshPortsBtn = document.getElementById("refreshPortsBtn");
const serverMessage = document.getElementById("serverMessage");

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
  connected: false,
  mockEnabled: false,
  serialPort: "",
  lastError: "",
  pollId: null,
  commandLoopId: null,
  sendingCommand: false,
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
const POLL_INTERVAL_MS = 250;
const AUTONOMOUS_SPEED = 150;
const AVOID_SPEED = 170;
const SAFE_DISTANCE_CM = 60;

connectBtn.addEventListener("click", onConnectClick);
modeSwitch.addEventListener("change", onModeChange);
refreshPortsBtn.addEventListener("click", refreshPorts);
mockSwitch.addEventListener("change", onMockToggle);

setupJoystick();
setupWinchButtons();
startPolling();
refreshPorts();
requestAnimationFrame(drawRadar);

async function onConnectClick() {
  if (state.mockEnabled) {
    setServerMessage("מצב מוק דאטא פעיל. כבה מוק כדי להתחבר לבקר אמיתי.", false);
    return;
  }

  if (state.connected) {
    await postJson("/api/disconnect", {});
    applyRemoteState(await fetchState());
    return;
  }

  const port = portSelect.value;
  if (!port) {
    await refreshPorts();
  }
  if (!portSelect.value) {
    setServerMessage("לא נמצא פורט COM זמין. ודא שהארדואינו מחובר למחשב.", true);
    return;
  }

  try {
    const response = await postJson("/api/connect", {
      port: portSelect.value,
      baudRate: 115200,
    });
    applyRemoteState(response.state);
    setServerMessage(`מחובר ל-${response.state.serialPort}.`, false);
  } catch (err) {
    setServerMessage(`חיבור נכשל: ${err.message}`, true);
  }
}

async function onMockToggle() {
  try {
    const response = await postJson("/api/mock", { enabled: mockSwitch.checked });
    applyRemoteState(response.state);
  } catch (err) {
    mockSwitch.checked = !mockSwitch.checked;
    setServerMessage(`החלפת מצב מוק נכשלה: ${err.message}`, true);
  }
}

function startPolling() {
  if (state.pollId) {
    clearInterval(state.pollId);
  }

  state.pollId = setInterval(async () => {
    try {
      applyRemoteState(await fetchState());
    } catch (err) {
      setServerMessage(`שגיאת שרת: ${err.message}`, true);
      setConnectedUI(false, "");
    }
  }, POLL_INTERVAL_MS);
}

function startCommandLoop() {
  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
  }

  state.commandLoopId = setInterval(async () => {
    if ((!state.connected && !state.mockEnabled) || state.sendingCommand) {
      return;
    }

    if (!state.manualMode) {
      updateAutonomousCommand();
    }

    state.sendingCommand = true;
    try {
      const response = await postJson("/api/command", state.cmd);
      applyRemoteState(response.state);
    } catch (err) {
      setServerMessage(`שליחת פקודה נכשלה: ${err.message}`, true);
    } finally {
      state.sendingCommand = false;
    }
  }, CONTROL_INTERVAL_MS);
}

function stopCommandLoop() {
  if (state.commandLoopId) {
    clearInterval(state.commandLoopId);
    state.commandLoopId = null;
  }
}

async function refreshPorts() {
  try {
    const response = await getJson("/api/ports");
    const selected = portSelect.value;
    portSelect.innerHTML = "";

    for (const port of response.ports) {
      const option = document.createElement("option");
      option.value = port;
      option.textContent = port;
      portSelect.appendChild(option);
    }

    if (selected && response.ports.includes(selected)) {
      portSelect.value = selected;
    }

    if (!response.ports.length) {
      const option = document.createElement("option");
      option.value = "";
      option.textContent = "אין פורטים";
      portSelect.appendChild(option);
    }
  } catch (err) {
    setServerMessage(`רענון פורטים נכשל: ${err.message}`, true);
  }
}

async function fetchState() {
  const response = await getJson("/api/state");
  return response.state;
}

function applyRemoteState(remoteState) {
  const wasConnected = state.connected;
  state.connected = Boolean(remoteState.connected);
  state.mockEnabled = Boolean(remoteState.mockEnabled);
  state.serialPort = remoteState.serialPort || "";
  state.lastError = remoteState.lastError || "";
  state.telemetry = {
    usRadar: remoteState.telemetry.usRadar,
    usFront: remoteState.telemetry.usFront,
    usLeft: remoteState.telemetry.usLeft,
    usRight: remoteState.telemetry.usRight,
    radarAngle: remoteState.telemetry.radarAngle,
  };
  state.cmd = {
    leftSpeed: remoteState.command.leftSpeed,
    rightSpeed: remoteState.command.rightSpeed,
    winchSpeed: remoteState.command.winchSpeed,
    radarAngle: remoteState.command.radarAngle,
  };
  state.lastRadarAngleSent = remoteState.telemetry.radarAngle ?? state.cmd.radarAngle;

  updateTelemetryUI();
  updateCommandUI();
  setConnectedUI(state.connected, state.serialPort);
  mockSwitch.checked = state.mockEnabled;
  portSelect.disabled = state.mockEnabled || state.connected;
  refreshPortsBtn.disabled = state.mockEnabled || state.connected;

  if (state.telemetry.usRadar !== null) {
    addRadarPoint(state.lastRadarAngleSent, state.telemetry.usRadar);
  }

  if (state.lastError) {
    setServerMessage(state.lastError, true);
  } else if (state.mockEnabled) {
    setServerMessage("מצב מוק דאטא פעיל - טלמטריה סימולטיבית בזמן אמת.", false);
  } else if (state.connected) {
    setServerMessage(`מחובר ל-${state.serialPort}.`, false);
  } else if (wasConnected && !state.connected) {
    setServerMessage("החיבור לבקר נותק.", true);
  }

  if ((state.connected || state.mockEnabled) && !state.commandLoopId) {
    startCommandLoop();
  }
  if (!state.connected && !state.mockEnabled) {
    stopCommandLoop();
  }
}

function setConnectedUI(connected, portName) {
  const active = connected || state.mockEnabled;
  connectBtn.disabled = state.mockEnabled;
  connectBtn.textContent = connected ? "נתק" : "חבר לבקר";
  connectionState.textContent = state.mockEnabled ? "מוק דאטא" : connected ? `מחובר ${portName}` : "לא מחובר";
  connectionState.classList.toggle("connected", active);
  connectionState.classList.toggle("disconnected", !active);
}

function updateTelemetryUI() {
  radarValue.textContent = formatDistance(state.telemetry.usRadar);
  frontValue.textContent = formatDistance(state.telemetry.usFront);
  leftValue.textContent = formatDistance(state.telemetry.usLeft);
  rightValue.textContent = formatDistance(state.telemetry.usRight);
  radarAngleValue.textContent = String(state.lastRadarAngleSent);
  radarDistanceValue.textContent = formatDistance(state.telemetry.usRadar, true);
}

function formatDistance(value, outLabel = false) {
  if (value === null || value === undefined) {
    return "--";
  }
  if (value === 999) {
    return outLabel ? "OUT" : "999";
  }
  return String(value);
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
    if (!state.joystick.active) {
      return;
    }
    state.joystick.active = false;
    if (event?.pointerId !== undefined) {
      joystickBase.releasePointerCapture(event.pointerId);
    }
    resetJoystick();
  };

  joystickBase.addEventListener("pointerup", stop);
  joystickBase.addEventListener("pointercancel", stop);
}

function updateManualCommandFromJoystick() {
  const forward = -state.joystick.y;
  const turn = state.joystick.x;

  state.cmd.leftSpeed = clamp(Math.round((forward + turn) * 255), -255, 255);
  state.cmd.rightSpeed = clamp(Math.round((forward - turn) * 255), -255, 255);
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

  if (usFront !== 999 && usFront < SAFE_DISTANCE_CM) {
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
  const now = Date.now();

  state.radarPoints.push({
    angleDeg,
    distanceCm: normalizedDistance,
    ts: now,
  });

  const cutoff = now - 7000;
  state.radarPoints = state.radarPoints.filter((point) => point.ts > cutoff);
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
    radarCtx.beginPath();
    radarCtx.moveTo(centerX, centerY);
    radarCtx.lineTo(centerX + Math.cos(rad) * maxRadius, centerY + Math.sin(rad) * maxRadius);
    radarCtx.stroke();
  }

  const now = Date.now();
  for (const point of state.radarPoints) {
    const alpha = Math.max(0.1, 1 - (now - point.ts) / 7000);
    const rad = toRadarRad(point.angleDeg);
    const r = (point.distanceCm / 340) * maxRadius;
    radarCtx.fillStyle = `rgba(0, 255, 163, ${alpha})`;
    radarCtx.beginPath();
    radarCtx.arc(centerX + Math.cos(rad) * r, centerY + Math.sin(rad) * r, 3, 0, Math.PI * 2);
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

function setServerMessage(message, isError) {
  serverMessage.textContent = message;
  serverMessage.style.color = isError ? "#b00020" : "";
}

async function getJson(url) {
  const response = await fetch(url, { cache: "no-store" });
  return parseJsonResponse(response);
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify(body),
  });
  return parseJsonResponse(response);
}

async function parseJsonResponse(response) {
  const payload = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(payload.error || `HTTP ${response.status}`);
  }
  return payload;
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}
