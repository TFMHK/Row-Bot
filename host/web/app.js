const connectBtn = document.getElementById("connectBtn");
const connectionState = document.getElementById("connectionState");
const modeSwitch = document.getElementById("modeSwitch");
const winchJoystickBase = document.getElementById("winchJoystickBase");
const winchJoystickHandle = document.getElementById("winchJoystickHandle");
const winchSpeedValue = document.getElementById("winchSpeedValue");
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
  lastRadarAngleSent: 0,
  radarSweepDir: 1,
  lastRadarStepTime: 0,
  manualMode: true,
  telemetry: {
    usRadar: null,
    usFront: null,
    usLeft: null,
    usRight: null,
    radarAngle: 0,
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
};

const sim = {
  maxRange: 340,
};

const CONTROL_INTERVAL_MS = 80;
const POLL_INTERVAL_MS = 500;
const AUTONOMOUS_SPEED = 150;
const AVOID_SPEED = 170;
const SAFE_DISTANCE_CM = 70;
// How often the servo advances one sweep step. Faster sweep => a full 0..90
// pass (which repaints every boat-relative slot) completes well within the TTL,
// so the radar picture stays filled instead of blinking.
const RADAR_STEP_INTERVAL_MS = 300;
// A slot's reading is trusted for this long after it was last measured. Long
// enough that a full sweep keeps every slot fresh, short enough that blips left
// behind by boat movement fade instead of forming phantom rings.
const RADAR_TTL_MS = 4000;

connectBtn.addEventListener("click", onConnectClick);
modeSwitch.addEventListener("change", onModeChange);
refreshPortsBtn.addEventListener("click", refreshPorts);
mockSwitch.addEventListener("change", onMockToggle);

setupJoystick();
setupWinchJoystick();
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

  if (!portSelect.value) {
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
    applyRemoteState(response.state, true);
    setServerMessage(`מחובר ל-${response.state.serialPort}.`, false);
  } catch (err) {
    setServerMessage(`חיבור נכשל: ${err.message}`, true);
  }
}

async function onMockToggle() {
  try {
    const response = await postJson("/api/mock", { enabled: mockSwitch.checked });
    applyRemoteState(response.state, true);
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
    const now = Date.now();
    if (now - state.lastRadarStepTime >= RADAR_STEP_INTERVAL_MS) {
      advanceRadarSweep();
      state.lastRadarStepTime = now;
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

function applyRemoteState(remoteState, syncCmd = false) {
  const wasConnected = state.connected;

  state.connected = Boolean(remoteState.connected);
  state.mockEnabled = Boolean(remoteState.mockEnabled);
  state.serialPort = remoteState.serialPort || "";
  state.lastError = remoteState.lastError || "";

  // Only overwrite the outgoing command on explicit connect / mock-enable.
  // During normal operation the browser is the source of truth for state.cmd;
  // overwriting it from poll responses or command echoes would discard in-flight
  // user input (e.g. winch button pressed between a poll and the 80ms command tick).
  if (syncCmd) {
    state.cmd = {
      leftSpeed: remoteState.command.leftSpeed,
      rightSpeed: remoteState.command.rightSpeed,
      winchSpeed: remoteState.command.winchSpeed,
      radarAngle: remoteState.command.radarAngle,
    };
  }

  // Always use backend telemetry (both real and mock modes).
  state.telemetry = {
    usRadar: remoteState.telemetry.usRadar,
    usFront: remoteState.telemetry.usFront,
    usLeft: remoteState.telemetry.usLeft,
    usRight: remoteState.telemetry.usRight,
    radarAngle: remoteState.telemetry.radarAngle,
  };
  state.lastRadarAngleSent = remoteState.telemetry.radarAngle ?? state.cmd.radarAngle;

  if (state.mockEnabled || state.connected) {
    updateRadarMemory();
  } else {
    radarMemory.clear();
  }

  updateTelemetryUI();
  updateCommandUI();
  setConnectedUI(state.connected, state.serialPort);

  mockSwitch.checked = state.mockEnabled;
  portSelect.disabled = state.mockEnabled || state.connected;
  refreshPortsBtn.disabled = state.mockEnabled || state.connected;

  if (state.lastError) {
    setServerMessage(state.lastError, true);
  } else if (state.mockEnabled) {
    setServerMessage("מוק דאטא: 4 חיישנים על סרבו מסתובב (0-90°), גופים אקראיים, תנועה לפי ג'ויסטיק", false);
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
  radarAngleValue.textContent = String(Math.round(state.lastRadarAngleSent));
  radarDistanceValue.textContent = formatDistance(state.telemetry.usFront, true);
}

function formatDistance(value, outLabel = false) {
  if (value === null || value === undefined) {
    return "--";
  }
  if (value === 999) {
    return outLabel ? "OUT" : "999";
  }
  return String(Math.round(value));
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

async function resetJoystick() {
  state.joystick.x = 0;
  state.joystick.y = 0;
  joystickStick.style.transform = "translate(-50%, -50%)";
  if (state.manualMode) {
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    try {
      const response = await postJson("/api/command", state.cmd);
      applyRemoteState(response.state);
    } catch (err) {
      console.error("Failed to send reset command:", err);
    }
  }
  updateCommandUI();
}

function setupWinchJoystick() {
  let active = false;

  const getClampedDy = (e) => {
    const rect = winchJoystickBase.getBoundingClientRect();
    const halfTravel = rect.height / 2 - 30; // 30 = half handle size
    const dy = e.clientY - (rect.top + rect.height / 2);
    return Math.max(-halfTravel, Math.min(halfTravel, dy));
  };

  const update = (dy) => {
    const rect = winchJoystickBase.getBoundingClientRect();
    const halfTravel = rect.height / 2 - 30;
    // Drag up (negative dy) = pull net (+speed); drag down = release (-speed)
    const speed = Math.round((-dy / halfTravel) * 255);
    state.cmd.winchSpeed = Math.max(-255, Math.min(255, speed));
    winchJoystickHandle.style.transform = `translate(-50%, calc(-50% + ${dy}px))`;
    winchSpeedValue.textContent = String(state.cmd.winchSpeed);
  };

  const stop = () => {
    if (!active) return;
    active = false;
    state.cmd.winchSpeed = 0;
    winchJoystickHandle.style.transform = "translate(-50%, -50%)";
    winchSpeedValue.textContent = "0";
  };

  winchJoystickBase.addEventListener("pointerdown", (e) => {
    active = true;
    winchJoystickBase.setPointerCapture(e.pointerId);
    update(getClampedDy(e));
  });
  winchJoystickBase.addEventListener("pointermove", (e) => {
    if (!active) return;
    update(getClampedDy(e));
  });
  winchJoystickBase.addEventListener("pointerup", stop);
  winchJoystickBase.addEventListener("pointercancel", stop);
}

// The 4 sensors sit 90° apart on ONE servo, so sweeping 0..90° already paints
// the full 360° picture. The PC drives the servo continuously.
function advanceRadarSweep() {
  const step = 15;
  let next = state.cmd.radarAngle + step * state.radarSweepDir;
  if (next >= 90) {
    next = 90;
    state.radarSweepDir = -1;
  } else if (next <= 0) {
    next = 0;
    state.radarSweepDir = 1;
  }
  state.cmd.radarAngle = next;
}

// Returns the nearest reading in radarMemory within toleranceDeg of a given
// boat-relative bearing. Because all 4 sensors rotate together with the servo,
// raw telemetry fields (usFront etc.) do NOT correspond to fixed hull directions;
// radarMemory stores each reading at its true bow-relative bearing.
function getMemoryDistance(bearingDeg, toleranceDeg) {
  let nearest = 999;
  const now = performance.now();
  for (const [slot, entry] of radarMemory) {
    if (
      entry.value != null &&
      now - entry.t <= RADAR_TTL_MS &&
      entry.value < nearest &&
      absAngleDiffDeg(slot, bearingDeg) <= toleranceDeg
    ) {
      nearest = entry.value;
    }
  }
  return nearest;
}

function updateAutonomousCommand() {
  const distFront = getMemoryDistance(0, 45);
  const distLeft  = getMemoryDistance(270, 45);
  const distRight = getMemoryDistance(90, 45);

  if (distFront < SAFE_DISTANCE_CM) {
    if (distLeft > distRight) {
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
}

function updateCommandUI() {
  leftSpeedValue.textContent = String(state.cmd.leftSpeed);
  rightSpeedValue.textContent = String(state.cmd.rightSpeed);
}

function drawRadar() {
  const w = radarCanvas.width;
  const h = radarCanvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const maxR = Math.min(w, h) * 0.43;

  radarCtx.clearRect(0, 0, w, h);
  radarCtx.fillStyle = "rgba(0, 20, 16, 0.9)";
  radarCtx.fillRect(0, 0, w, h);

  radarCtx.strokeStyle = "rgba(0, 255, 180, 0.25)";
  radarCtx.lineWidth = 1;

  for (let i = 1; i <= 4; i += 1) {
    radarCtx.beginPath();
    radarCtx.arc(cx, cy, (maxR / 4) * i, 0, Math.PI * 2);
    radarCtx.stroke();
  }

  for (let deg = 0; deg < 360; deg += 30) {
    const rad = degToRad(deg);
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.lineTo(cx + Math.sin(rad) * maxR, cy - Math.cos(rad) * maxR);
    radarCtx.stroke();
  }

  drawBoat(cx, cy);
  drawSensorArcs(cx, cy, maxR);
  drawSensorBeams(cx, cy, maxR);

  requestAnimationFrame(drawRadar);
}

function drawBoat(cx, cy) {
  radarCtx.save();
  radarCtx.translate(cx, cy);
  // Boat always points up - no rotation
  radarCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  radarCtx.beginPath();
  radarCtx.moveTo(0, -13);
  radarCtx.lineTo(9, 10);
  radarCtx.lineTo(0, 6);
  radarCtx.lineTo(-9, 10);
  radarCtx.closePath();
  radarCtx.fill();
  radarCtx.restore();
}

// All 4 ultrasonic sensors sit 90° apart on ONE servo axis and rotate together.
// dir = their bow-relative bearing at servo home (0); the live bearing adds the
// current servo angle (radarAngle). Boat is drawn pointing up (front = 0°).
const SENSOR_BEAMS = [
  { dir: 0, key: "usFront", color: "rgba(44, 255, 197, 1)" },
  { dir: 90, key: "usRight", color: "rgba(255, 150, 100, 1)" },
  { dir: 180, key: "usRadar", color: "rgba(200, 120, 255, 1)" },
  { dir: 270, key: "usLeft", color: "rgba(100, 200, 255, 1)" },
];

// Radar persistence: each boat-relative angle slot keeps its last scan for a
// short while (RADAR_TTL_MS). Fresh sweeps refresh it; stale readings expire so
// that blips left behind by boat movement/rotation don't linger as phantom
// rings or arcs that never existed in the world.
const radarMemory = new Map();

function updateRadarMemory() {
  // The whole sensor array is rotated by the current servo angle.
  const sweep = state.telemetry.radarAngle ?? 0;
  const now = performance.now();
  for (const beam of SENSOR_BEAMS) {
    const slot = normalizeDeg(beam.dir + sweep);
    const newVal = state.telemetry[beam.key];
    if (newVal == null) continue;
    // Always refresh the timestamp when this slot is actually measured, so a
    // continuously confirmed obstacle stays alive; keep the smoothed value to
    // avoid flicker from minor (<5 cm) sensor noise.
    const existing = radarMemory.get(slot);
    const value =
      existing && Math.abs(newVal - existing.value) <= 5 ? existing.value : newVal;
    radarMemory.set(slot, { value, t: now });
  }
}

// Remove slots whose last real measurement is older than the TTL.
function pruneRadarMemory() {
  const now = performance.now();
  for (const [slot, entry] of radarMemory) {
    if (now - entry.t > RADAR_TTL_MS) {
      radarMemory.delete(slot);
    }
  }
}

function drawSensorArcs(cx, cy, maxR) {
  pruneRadarMemory();

  // Collect the currently valid detections as points at their true
  // (bearing, distance) location, so a straight wall lands on a straight line
  // and scattered bodies stay separate dots.
  const points = [];
  for (const [slot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999) continue;
    const pixelDist = (entry.value / sim.maxRange) * maxR;
    const rad = degToRad(slot) - Math.PI / 2;
    points.push({
      slot,
      dist: entry.value,
      x: cx + Math.cos(rad) * pixelDist,
      y: cy + Math.sin(rad) * pixelDist,
    });
  }

  // Connect neighbours that belong to the same continuous surface (a wall):
  // adjacent bearing slots (<= ~20° apart) whose ranges are close. Grazing-angle
  // steps along a flat wall grow with 1/cos, so the range gap between two 15°
  // slots can reach ~55 cm near the wall edges; 70 cm covers that while still
  // leaving well-separated bodies as isolated dots.
  points.sort((a, b) => a.slot - b.slot);
  radarCtx.strokeStyle = "rgba(44, 255, 197, 0.55)";
  radarCtx.lineWidth = 2;
  for (let i = 0; i < points.length; i += 1) {
    const a = points[i];
    const b = points[(i + 1) % points.length];
    if (points.length < 2) break;
    if (absAngleDiffDeg(a.slot, b.slot) <= 20 && Math.abs(a.dist - b.dist) <= 45) {
      radarCtx.beginPath();
      radarCtx.moveTo(a.x, a.y);
      radarCtx.lineTo(b.x, b.y);
      radarCtx.stroke();
    }
    if (points.length === 2) break; // avoid drawing the same pair twice
  }

  // Draw each detection as a small blip on top of the connecting lines.
  radarCtx.fillStyle = "rgba(44, 255, 197, 0.95)";
  for (const p of points) {
    radarCtx.beginPath();
    radarCtx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
    radarCtx.fill();
  }
}

function drawSensorBeams(cx, cy, maxR) {
  const fovHalf = 7.5;
  const sweep = state.telemetry.radarAngle ?? 0;
  radarCtx.save();
  radarCtx.lineWidth = 1.5;
  for (const beam of SENSOR_BEAMS) {
    const startRad = degToRad(beam.dir + sweep - fovHalf);
    const endRad = degToRad(beam.dir + sweep + fovHalf);
    const centerRad = degToRad(beam.dir + sweep);

    // Soft FOV fill
    radarCtx.fillStyle = beam.color.replace(", 1)", ", 0.08)");
    radarCtx.beginPath();
    radarCtx.moveTo(cx, cy);
    radarCtx.arc(cx, cy, maxR, startRad - Math.PI / 2, endRad - Math.PI / 2);
    radarCtx.closePath();
    radarCtx.fill();

    // FOV edges + center
    radarCtx.strokeStyle = beam.color.replace(", 1)", ", 0.6)");
    for (const rad of [startRad, endRad, centerRad]) {
      radarCtx.beginPath();
      radarCtx.moveTo(cx, cy);
      radarCtx.lineTo(cx + Math.sin(rad) * maxR, cy - Math.cos(rad) * maxR);
      radarCtx.stroke();
    }
  }
  radarCtx.restore();
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

function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

function normalizeDeg(deg) {
  let out = deg % 360;
  if (out < 0) {
    out += 360;
  }
  return out;
}

function absAngleDiffDeg(a, b) {
  let d = normalizeDeg(a) - normalizeDeg(b);
  d = ((d + 540) % 360) - 180;
  return Math.abs(d);
}

