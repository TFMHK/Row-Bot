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

const joystickBase = document.getElementById("joystickBase");
const joystickStick = document.getElementById("joystickStick");

const radarCanvas = document.getElementById("radarCanvas");
const radarCtx = radarCanvas.getContext("2d");

const worldCanvas = document.getElementById("worldCanvas");
const worldCtx = worldCanvas.getContext("2d");
const worldPosValue = document.getElementById("worldPosValue");
const worldInRangeValue = document.getElementById("worldInRangeValue");

const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
const mapPointsValue = document.getElementById("mapPointsValue");
const mapClearBtn = document.getElementById("mapClearBtn");

const state = {
  connected: false,
  mockEnabled: false,
  serialPort: "",
  lastError: "",
  pollId: null,
  eventSource: null,
  commandLoopId: null,
  sendingCommand: false,
  lastRadarAngleSent: 0,
  radarSweepDir: 1,
  lastRadarStepTime: 0,
  manualMode: true,
  // Committed autonomous avoidance turn: 0 = none, -1 = turning left, +1 = right.
  avoidDir: 0,
  // אודומטריה צד-לקוח (dead-reckoning): אומדן פוזה מתוך אינטגרציה של אותן
  // פקודות המנועים שאנו שולחים, באותו מודל קינמטי כמו הסירה/הסימולטור. זה מה
  // שחומרה אמיתית תעשה (אין GPS/מצפן על החוט), ולכן כל מחסנית התפיסה+הניווט רצה
  // על פוזה משוערת בסימולטור ובמציאות גם יחד — אם זה עובד בסימולטור, זה יעבוד במים.
  // (בסימולטור האומדן עוקב אחר אמת-השרת; boatX/Y/heading משמשים רק לתצוגת אמת לדיבאג.)
  pose: { x: 0, y: 0, headingDeg: 0, speedCms: 0 },
  lastPoseT: 0,
  // ניווט מוכוון-יעד (הגעה לקצה השני של הבריכה). מכונת מצבים:
  //   "seek"    - follow-the-gap מוטה אל היעד
  //   "follow"  - עקיבת קו-מתאר של מכשול (Bug2) לחילוץ ממלכודות
  //   "arrived" - הגענו ליעד, עוצרים
  nav: {
    mode: "seek",
    goal: null,            // {x, y} בקואורדינטות עולם
    followHand: 1,         // +1 = שמור מכשול מימין, -1 = משמאל
    wallSince: 0,
    progressAnchor: null,  // {dist, t} תצלום מרחק-ליעד לגילוי תקיעה
  },
  telemetry: {
    usRadar: null,
    usFront: null,
    usLeft: null,
    usRight: null,
    radarAngle: 0,
    boatX: 0,
    boatY: 0,
    boatHeadingDeg: 0,
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
  maxRange: 450,
};

// Ground-truth mock world (obstacle bodies + boundary), fetched once per mock
// session so the world view can be compared against the radar picture.
const world = {
  loaded: false,
  loading: false,
  boundsHalfX: 600,
  boundsHalfY: 450,
  maxRange: 450,
  targets: [],
};

const CONTROL_INTERVAL_MS = 80;
const POLL_INTERVAL_MS = 500;

// --- מודל אודומטריה (dead-reckoning) — זהה למודל הפיזיקלי של _mock_loop בשרת ---
// על ידי אינטגרציה של אותן פקודות שאנו שולחים מקבלים אומדן פוזה הזהה למה שהסירה
// האמיתית תעשה. הערכים חייבים להיות זההים לשרת כדי שהאומדן יישאר צמוד לאמת בסימולטור.
const DR_MAX_SPEED_CMS = 45;          // target_speed = throttle * 45 (גבול פיזיקלי)
const DR_ACCEL_GAIN = 0.24;           // החלקת תאוצה לעבר מהירות היעד
const DR_TURN_RATE = Math.PI * 0.25;  // rad/s בסיבוב מלא
// מהירות שיוט אוטונומית. מכוונת בכוונה נמוכה יחסית כדי שהמכ"ם הסורק (מסתובב על
// סרבו, מכסה את המעגל ב~1 שנייה) תמיד "מדביק" גופים שנכנסים לחזית לפני מגע —
// במהירות גבוהה הסירה עלולה לחצות פער-סריקה ולהיכנס לגוף שטרם נסרק.
const AUTONOMOUS_SPEED = 135;
// Slowest forward speed while threading past a nearby obstacle. Easing down to
// this as the bow closes in gives the servo sweep time to refresh the picture
// and lets the turn actually bite before impact.
const AUTONOMOUS_MIN_SPEED = 75;
const AVOID_SPEED = 170;
// Below this front distance the boat commits to spinning in place toward open
// water. It keeps spinning (hysteresis) until the bow is clear past
// CLEAR_DISTANCE_CM, so it never chatters on/off right at one threshold.
const SAFE_DISTANCE_CM = 70;
const CLEAR_DISTANCE_CM = 130;
// While cruising, start gently steering away from an obstacle this far ahead so
// the boat arcs around it smoothly instead of charging up and hard-spinning.
const STEER_LOOKAHEAD_CM = 220;
const STEER_MAX = 120;
// --- ניווט יזום לעבר מרחב פתוח ("follow the gap") ---
// במקום לחכות שהחרטום ייחסם ואז להתחמק, הניווט סורק את קשת החזית ובוחר בכל רגע
// את הכיוון עם המרחק הפנוי הגדול ביותר (הכי פחות צפוף) — כולל התחשבות בקירות —
// ומטה את הסירה לעברו. כך היא נמשכת אל מים פתוחים ונמנעת מלהיכנס לצבירי גופים
// מלכתחילה, במקום להסתבך בתוכם ולהיתקע.
const OPEN_SCAN_DEG = 85;        // סורקים ±זווית זו סביב החרטום (קשת חזית)
const OPEN_SCAN_STEP = 10;       // צעד הסריקה במעלות
const OPEN_SCAN_TOL = 12;        // סובלנות זוויתית לכל כיוון-מועמד (חופף לצעד)
const OPEN_CLEAR_CAP = 320;      // ס"מ - מעבר לזה כיוון "פתוח מספיק" (לא מתגמל עוד)
const OPEN_TURN_PENALTY = 0.5;   // ס"מ קנס לכל מעלת סטייה מהחרטום (מאזן בין
                                 // "להמשיך ישר" ל"לפנות אל הפתוח") — פונה חזק רק
                                 // אם הכיוון הפתוח פנוי משמעותית יותר.
const OPEN_STEER_GAIN = 1.5;     // ממיר זווית-יעד (מעלות) לעוצמת היגוי

// --- ניווט מוכוון-יעד: הגעה לקצה השני של הבריכה ---
// המוצא קרוב לאחת הדפנות; היעד הוא הדופן שממול. הסירה נמשכת אל היעד (איבר משיכה
// ב-follow-the-gap), ואם היא נתקעת (חוסר התקדמות נטו) היא עוברת לעקיבת קו-מתאר
// בסגנון Bug2 שמובטח שיחלץ אותה ממלכודות קעורות/פינות — ואז חוזרת לשיוט אל היעד.
const GOAL_MARGIN_CM = 60;        // עד כמה מהקיר הרחוק ממקמים את נקודת היעד
const GOAL_ARRIVE_CM = 90;        // מרחק מהיעד שנחשב "הגענו" -> עצירה
const GOAL_ATTRACT_PENALTY = 0.9; // ס"מ קנס לכל מעלת סטייה מכיוון היעד (מחליף את
                                  // OPEN_TURN_PENALTY במצב seek -> מטה אל היעד)
// גלאי-תקיעה: אם ההתקדמות נטו אל היעד קטנה מ-STUCK_PROGRESS_CM בתוך חלון הזמן,
// מניחים שנתקענו במינימום מקומי ועוברים לעקיבת-דופן.
const STUCK_WINDOW_MS = 4000;
const STUCK_PROGRESS_CM = 40;
// מצב עקיבת-דופן (Bug2): בקר-P שומר מרחק-סף קבוע מהדופן/גוף עד שכיוון היעד נפתח.
const WALL_FOLLOW_STANDOFF_CM = 90;
const WALL_FOLLOW_SPEED = 120;
const WALL_FOLLOW_GAIN = 1.2;     // המרה משגיאת-מרחק (ס"מ) לעוצמת היגוי
const WALL_LEAVE_CLEAR_CM = 200;  // כיוון היעד נחשב "פתוח" מעל מרחק פנוי זה -> עזיבה
// How often the servo advances one sweep step. The four sensors are only 15°
// wide and sit 90° apart, so at any instant they cover just 4×15°=60° of the
// full circle — everything else is a momentary blind gap. The ONLY thing that
// fills those gaps is the sweep, so it must be fast: a body dead ahead has to be
// swept over (and thus seen) before the boat can reach it. At 80ms/step a full
// 0..90..0 servo cycle (~12 steps) scans the entire 360° in ~1s, which at cruise
// speed is only ~60cm of travel — tight enough that head-on bodies are caught
// well before contact. (Slower sweeps let the boat charge into gaps unseen.)
const RADAR_STEP_INTERVAL_MS = 80;
// A slot's reading is trusted for this long after it was last measured. A full
// 0..90..0 sweep cycle (~12 steps × RADAR_STEP_INTERVAL_MS ≈ 1 s) refreshes every
// bearing at least once, and the turnaround slots (0° and 90°) once per cycle, so
// the TTL only needs to comfortably exceed that cycle time (plus network/servo
// lag) — 5 s leaves a very wide margin so no slot ever expires before its refresh.
const RADAR_TTL_MS = 5000;

// שכבת הגנה והתחמקות אקטיבית
const AVOID_SAFETY_RADIUS = 45;  // ס"מ - קו אדום, אסור לסירה להיות במרחק כזה ממכשול
const AVOID_WARNING_RADIUS = 80; // ס"מ - טווח הדיפה, כניסה אליו תייצר פקודת היגוי נגדית
// מושל מהירות קדימה: אסור לדהור לעבר מכשול שזוהה מהר מכפי שאפשר לפנות ממנו.
// המהירות קדימה נחתכת כפונקציה של המרחק הפנוי בחרוט שלפני החרטום, כך שגוף חזיתי
// דוחף את המצערת לאפס הרבה לפני מגע — הסירה עדיין יכולה להסתובב במקום ולחפש פתח,
// אבל פיזית אינה יכולה להתקדם לתוך גוף. פועל גם בשליטה ידנית וגם באוטונומית.
const GOVERNOR_RANGE_CM = 150;    // ס"מ - מתחילים להאט מהמרחק הזה
const GOVERNOR_STOP_CM = 48;      // ס"מ - מהירות קדימה מתאפסת במרחק (משטח) הזה
const GOVERNOR_FWD_CONE_DEG = 32; // חרוט צר סביב החרטום בלבד — רק מכשול כמעט-חזיתי
                                  // מגביל מהירות. כל ההיגוי (חיפוש מרחב פתוח
                                  // והתחמקות) מטופל ע"י שכבת הניווט, כך שהמושל לא
                                  // כופה כיוון פנייה משלו ולא נאבק עם הניווט.

// Accumulated radar map: unlike the boat-centric radar view, this builds a
// persistent picture in FIXED world coordinates. Each valid sensor reading is
// projected to its absolute (world) hit point using the boat's dead-reckoned
// pose, then quantised into a grid cell so memory stays bounded while repeated
// hits on the same spot raise that cell's confidence.
const MAP_CELL_CM = 12;
const MAP_MAX_CELLS = 4000;
const MAP_TRAIL_MAX = 600;
// A sonar returns a range within a ~15° beam, so one reading is an ARC of
// possible obstacle positions — not a point. We paint the whole arc; a cell is
// trusted as a real obstacle only where arcs from different bearings/positions
// INTERSECT (>= MAP_CONFIRM_HITS). A lone arc leaves each cell well below the
// threshold, so phantom single-beam echoes never register.
const MAP_ARC_FOV_DEG = 7.5;   // half-width of the sensor beam (matches the sim)
const MAP_ARC_STEP_DEG = 2.5;  // angular sampling resolution along the arc
const MAP_HIT_CAP = 15;        // bound per-cell confidence so clearing can erode it
const MAP_CONFIRM_HITS = 2;    // intersecting arc weight required to trust a cell
// The farther a reading is, the wider its 15° beam smears across the grid, so a
// lone far echo might be a single stray point spread over many cells. Weight
// each arc's per-cell contribution DOWN with range: full strength within
// MAP_ARC_REF_CM, then inverse-distance beyond, so near/tight arcs count for
// more than far/sprawling ones.
const MAP_ARC_REF_CM = 100;    // range at which an arc contributes weight 1.0
// A near beam barely smears (15° at 40cm is under one cell), so a single near
// arc is already well-localised and trustworthy — no need to wait for a second
// intersecting arc. Let close arcs earn up to this weight so anything within
// ~MAP_ARC_REF_CM/2 (~50cm) confirms in ONE look, while far arcs stay < 1.0 and
// must still intersect. This is what makes near obstacles register promptly.
const MAP_ARC_MAX_WEIGHT = 2;  // >= MAP_CONFIRM_HITS so a single near arc confirms
// A no-echo (or free ray) doesn't erase a cell outright — it FADES it a little,
// since repeated non-detections steadily raise the odds those points were noise.
const MAP_CLEAR_DECAY = 0.5;   // gentle weight erosion per free/no-echo pass
const mapCells = new Map();
const boatTrail = [];
// Dedupe identical arc observations (same beam, servo step, boat cell, range) so
// a stationary boat re-seeing the same arc every frame can't inflate a lone arc
// above the intersection threshold. Each DISTINCT look counts as exactly one arc.
const lastArcObs = new Map();

connectBtn.addEventListener("click", onConnectClick);
modeSwitch.addEventListener("change", onModeChange);
refreshPortsBtn.addEventListener("click", refreshPorts);
mockSwitch.addEventListener("change", onMockToggle);
mapClearBtn.addEventListener("click", clearRadarMap);

setupJoystick();
setupWinchJoystick();
startTelemetryStream();
refreshPorts();
requestAnimationFrame(drawRadar);
requestAnimationFrame(drawWorld);
requestAnimationFrame(drawMap);

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

// Prefer a Server-Sent Events push stream: the backend emits a fresh snapshot
// the instant new telemetry is parsed, so the UI reflects the boat in near real
// time instead of lagging behind a 500ms poll. Falls back to polling if the
// browser lacks EventSource or the stream endpoint can't be reached.
function startTelemetryStream() {
  if (typeof window.EventSource === "undefined") {
    startPolling();
    return;
  }

  if (state.eventSource) {
    state.eventSource.close();
    state.eventSource = null;
  }

  try {
    const es = new EventSource("/api/stream");
    state.eventSource = es;

    es.onmessage = (event) => {
      // A live stream frame means the poll fallback (if any) is redundant.
      if (state.pollId) {
        clearInterval(state.pollId);
        state.pollId = null;
      }
      try {
        const payload = JSON.parse(event.data);
        if (payload && payload.state) {
          applyRemoteState(payload.state);
        }
      } catch (err) {
        // Ignore malformed frames; the next one will resync the UI.
      }
    };

    es.onerror = () => {
      // EventSource reconnects on its own; keep a slow poll running meanwhile
      // so the UI still updates if the stream stays down (e.g. server restart).
      if (!state.pollId) {
        startPolling();
      }
    };
  } catch (err) {
    startPolling();
  }
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

    // === יירוט ודריסת פקודות היגוי לצורך בטיחות ===
    // הדריסה מחושבת על עותק בלבד, כך שברגע שיוצאים מהטווח הכתום
    // השליטה חוזרת מיד לניווט האוטונומי/השליטה הידנית.
    const outgoing = computeSafeCommand();

    // אודומטריה: מקדמים את אומדן הפוזה לפי הפקודה הנשלחת, כך שהתפיסה+הניווט
    // רצים על פוזה משוערת (ריאל-טרנספרבילית) ולא על אמת-שרת שקיימת רק בסימולטור.
    const poseNow = performance.now();
    integratePose(outgoing, state.lastPoseT ? (poseNow - state.lastPoseT) / 1000 : 0);
    state.lastPoseT = poseNow;

    state.sendingCommand = true;
    try {
      const response = await postJson("/api/command", outgoing);
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

// Load the ground-truth mock world once per mock session. Targets are randomised
// each time mock mode is (re)enabled, so we refetch whenever mock turns on.
async function ensureWorldLoaded() {
  if (!state.mockEnabled) {
    world.loaded = false;
    world.targets = [];
    return;
  }
  if (world.loaded || world.loading) return;
  world.loading = true;
  try {
    const w = await getJson("/api/world");
    world.boundsHalfX = w.boundsHalfX ?? world.boundsHalfX;
    world.boundsHalfY = w.boundsHalfY ?? world.boundsHalfY;
    world.maxRange = w.maxRange ?? world.maxRange;
    world.targets = Array.isArray(w.targets) ? w.targets : [];
    world.loaded = true;
  } catch (err) {
    // Non-fatal: world view simply stays empty until the next attempt.
  } finally {
    world.loading = false;
  }
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
    // Fresh connect / mock-enable => start the accumulated map from scratch so
    // stale detections from a previous (possibly re-randomised) world are gone.
    clearRadarMap();
    resetPose();
  }

  // Always use backend telemetry (both real and mock modes).
  state.telemetry = {
    usRadar: remoteState.telemetry.usRadar,
    usFront: remoteState.telemetry.usFront,
    usLeft: remoteState.telemetry.usLeft,
    usRight: remoteState.telemetry.usRight,
    radarAngle: remoteState.telemetry.radarAngle,
    boatX: remoteState.telemetry.boatX ?? 0,
    boatY: remoteState.telemetry.boatY ?? 0,
    boatHeadingDeg: remoteState.telemetry.boatHeadingDeg ?? 0,
  };
  state.lastRadarAngleSent = remoteState.telemetry.radarAngle ?? state.cmd.radarAngle;

  if (state.mockEnabled || state.connected) {
    updateRadarMemory();
    accumulateRadarMap();
  } else {
    radarMemory.clear();
  }

  ensureWorldLoaded();

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
  state.avoidDir = 0;
  // מאתחלים מחדש את היעד עם כניסה מחודשת למצב אוטונומי (נקבע מהמיקום העדכני).
  state.nav.goal = null;
  state.nav.mode = "seek";
  state.nav.progressAnchor = null;
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

// The 4 sensors sit 90° apart on ONE servo, so sweeping 15..90° already paints
// the full 360° picture. The PC drives the servo so it climbs to 90°, then
// walks gently back down one 15° step at a time to 0° (instead of slamming
// straight from 90° to 0° in a single move, which would stress the servo
// gears). At 0° it resumes climbing. The sweep MUST reach 0° so the four
// sensors (mounted at 0/90/180/270°) actually sample the cardinal directions —
// dead ahead most of all. Stopping at 15° left a permanent blind spot at the
// bow, so head-on obstacles went unseen until the boat had already hit them.
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

// Returns the nearest obstacle distance (cm) within toleranceDeg of a given
// bow-relative bearing, taken from the motion-compensated accumulated world map
// (confirmed cells) plus fresh live radar. The map is keyed in fixed world
// coordinates, so a clearance stays correct as the boat closes in; the raw
// per-slot radar range alone goes stale between the slow servo sweeps, which is
// what made the navigator charge into walls it "remembered" as far away.
function getMemoryDistance(bearingDeg, toleranceDeg) {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const targetAbs = normalizeDeg(heading + bearingDeg);
  let nearest = 999;

  const consider = (px, py) => {
    const d = Math.hypot(px - bx, py - by);
    if (d >= nearest) return;
    const absBearing = (Math.atan2(px - bx, py - by) * 180) / Math.PI;
    if (absAngleDiffDeg(absBearing, targetAbs) <= toleranceDeg) nearest = d;
  };

  // Confirmed obstacles in the accumulated world map (arcs from several bearings
  // intersected here), positioned in world space so the range is never stale.
  for (const cell of mapCells.values()) {
    if (getCellConfidence(cell) >= MAP_CONFIRM_HITS) consider(cell.x, cell.y);
  }

  // Fresh live radar too, so a newly seen obstacle triggers avoidance before the
  // map has had two sweeps to confirm it.
  const now = performance.now();
  for (const [absSlot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999 || now - entry.t > RADAR_TTL_MS) continue;
    const rad = degToRad(absSlot);
    consider(bx + Math.sin(rad) * entry.value, by + Math.cos(rad) * entry.value);
  }

  return nearest;
}

// dir < 0 turns left (heading falls), dir > 0 turns right (heading rises).
// turn = (right - left) in the sim, so right > left => turn right.
function applyAvoidTurn(dir) {
  state.cmd.leftSpeed = -dir * AVOID_SPEED;
  state.cmd.rightSpeed = dir * AVOID_SPEED;
}

// מקדם את אומדן הפוזה לפי הפקודה שאנו עומדים לשלוח (דווקא מהפקודה, לא מטלמטריה)
// — בדיוק כמו שחומרה אמיתית תעשה. מודל זהה ל-_mock_loop בשרת.
function integratePose(cmd, dtSec) {
  const dt = clamp(dtSec, 0, 0.2);
  if (dt <= 0) return;
  const throttle = (cmd.leftSpeed + cmd.rightSpeed) / (2 * 255);
  const turn = (cmd.rightSpeed - cmd.leftSpeed) / (2 * 255);
  const target = throttle * DR_MAX_SPEED_CMS;
  state.pose.speedCms += (target - state.pose.speedCms) * DR_ACCEL_GAIN * dt / 0.05;
  const hr = degToRad(state.pose.headingDeg) + turn * DR_TURN_RATE * dt;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;
  state.pose.x = clamp(state.pose.x + Math.sin(hr) * state.pose.speedCms * dt, -HX, HX);
  state.pose.y = clamp(state.pose.y + Math.cos(hr) * state.pose.speedCms * dt, -HY, HY);
  state.pose.headingDeg = normalizeDeg((hr * 180) / Math.PI);
}

// מאפס את מסגרת האודומטריה (עם איפוס העולם בסימולטור, כדי שהאומדן יישאר מיושר לאמת).
function resetPose() {
  state.pose.x = 0;
  state.pose.y = 0;
  state.pose.headingDeg = 0;
  state.pose.speedCms = 0;
  state.lastPoseT = 0;
}

// Distance (cm) from the boat to the rectangular world boundary along a world
// bearing. World coords use x = sin(bearing), y = cos(bearing) like the rest of
// the code. Lets the open-water seeker treat walls as "density" too, so it
// steers toward the open interior instead of driving into a corner.
function distanceToWall(bx, by, absBearingDeg, HX, HY) {
  const rad = degToRad(absBearingDeg);
  const dx = Math.sin(rad);
  const dy = Math.cos(rad);
  let t = Infinity;
  if (dx > 1e-6) t = Math.min(t, (HX - bx) / dx);
  else if (dx < -1e-6) t = Math.min(t, (-HX - bx) / dx);
  if (dy > 1e-6) t = Math.min(t, (HY - by) / dy);
  else if (dy < -1e-6) t = Math.min(t, (-HY - by) / dy);
  return t < 0 ? 0 : t;
}

// Proactive open-water seeking. Scans the forward arc and returns the bow-
// relative bearing with the greatest clearance (nearest body OR wall), lightly
// penalised for deviating from a REFERENCE bearing so the boat still makes
// progress. With refBearing = the goal direction the boat is pulled toward the
// least-dense direction that also advances toward the goal, instead of merely
// wandering into open water.
function chooseOpenHeading(refBearing = 0, penalty = OPEN_TURN_PENALTY) {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;

  let bestBearing = 0;
  let bestScore = -Infinity;
  for (let b = -OPEN_SCAN_DEG; b <= OPEN_SCAN_DEG; b += OPEN_SCAN_STEP) {
    const obstacle = getMemoryDistance(b, OPEN_SCAN_TOL);
    const wall = distanceToWall(bx, by, normalizeDeg(heading + b), HX, HY);
    const clear = Math.min(obstacle, wall, OPEN_CLEAR_CAP);
    const score = clear - penalty * Math.abs(b - refBearing);
    if (score > bestScore) {
      bestScore = score;
      bestBearing = b;
    }
  }
  return bestBearing;
}

// ---- \u05e0\u05d9\u05d5\u05d5\u05d8 \u05de\u05d5\u05db\u05d5\u05d5\u05df-\u05d9\u05e2\u05d3: \u05e2\u05d6\u05e8\u05d9\u05dd ----

// \u05e7\u05d5\u05d1\u05e2 \u05d0\u05ea \u05d4\u05d9\u05e2\u05d3 \u05e2\u05dd \u05db\u05e0\u05d9\u05e1\u05d4 \u05dc\u05de\u05e6\u05d1 \u05d0\u05d5\u05d8\u05d5\u05e0\u05d5\u05de\u05d9: \u05d4\u05d3\u05d5\u05e4\u05df \u05d4\u05e7\u05e8\u05d5\u05d1\u05d4 \u05d1\u05d9\u05d5\u05ea\u05e8 \u05d4\u05d9\u05d0 \u05d4\u05de\u05d5\u05e6\u05d0,
// \u05d5\u05d4\u05d9\u05e2\u05d3 \u05d4\u05d5\u05d0 \u05d4\u05d3\u05d5\u05e4\u05df \u05e9\u05de\u05de\u05d5\u05dc \u05d1\u05d0\u05d5\u05ea\u05d4 \u05e7\u05d5\u05d0\u05d5\u05e8\u05d3\u05d9\u05e0\u05d8\u05d4 \u05e6\u05d3\u05d3\u05d9\u05ea ("\u05d4\u05e7\u05e6\u05d4 \u05d4\u05e9\u05e0\u05d9").
function initNavGoal() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const HX = world.boundsHalfX || 600;
  const HY = world.boundsHalfY || 450;
  const candidates = [
    { d: bx + HX, goal: { x:  HX - GOAL_MARGIN_CM, y: by } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05e9\u05de\u05d0\u05dc -> \u05d9\u05e2\u05d3 \u05d9\u05de\u05d9\u05df
    { d: HX - bx, goal: { x: -HX + GOAL_MARGIN_CM, y: by } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05d9\u05de\u05d9\u05df -> \u05d9\u05e2\u05d3 \u05e9\u05de\u05d0\u05dc
    { d: by + HY, goal: { x: bx, y:  HY - GOAL_MARGIN_CM } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05ea\u05d7\u05ea\u05d5\u05e0\u05d4 -> \u05d9\u05e2\u05d3 \u05e2\u05dc\u05d9\u05d5\u05df
    { d: HY - by, goal: { x: bx, y: -HY + GOAL_MARGIN_CM } }, // \u05de\u05d5\u05e6\u05d0 \u05dc\u05d9\u05d3 \u05d3\u05d5\u05e4\u05df \u05e2\u05dc\u05d9\u05d5\u05e0\u05d4 -> \u05d9\u05e2\u05d3 \u05ea\u05d7\u05ea\u05d5\u05df
  ];
  candidates.sort((a, b) => a.d - b.d);
  state.nav.goal = candidates[0].goal;
  state.nav.mode = "seek";
  state.nav.progressAnchor = null;
  state.avoidDir = 0;
}

// \u05de\u05e8\u05d7\u05e7 \u05e0\u05d5\u05db\u05d7\u05d9 \u05de\u05d4\u05e1\u05d9\u05e8\u05d4 \u05dc\u05d9\u05e2\u05d3 (\u05e1"\u05de).
function goalDistance() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const g = state.nav.goal;
  return Math.hypot(g.x - bx, g.y - by);
}

// \u05d4\u05d0\u05d6\u05d9\u05de\u05d5\u05ea \u05d4\u05d9\u05d7\u05e1\u05d9 \u05dc\u05d7\u05e8\u05d8\u05d5\u05dd (\u2011180..180, 0 = \u05d9\u05e9\u05e8 \u05e7\u05d3\u05d9\u05de\u05d4) \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3.
function goalBearingRel() {
  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;
  const g = state.nav.goal;
  const absBearing = (Math.atan2(g.x - bx, g.y - by) * 180) / Math.PI;
  return ((absBearing - heading + 540) % 360) - 180;
}

// \u05d2\u05dc\u05d0\u05d9-\u05ea\u05e7\u05d9\u05e2\u05d4: \u05e2\u05d5\u05e7\u05d1 \u05d0\u05d7\u05e8 \u05d4\u05ea\u05e7\u05d3\u05de\u05d5\u05ea \u05e0\u05d8\u05d5 \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3 \u05d1\u05d7\u05dc\u05d5\u05df \u05d6\u05de\u05df \u05e0\u05e2. \u05d0\u05dd \u05d1\u05de\u05e6\u05d1 seek
// \u05dc\u05d0 \u05d4\u05ea\u05e7\u05d3\u05de\u05e0\u05d5 \u2014 \u05e2\u05d5\u05d1\u05e8\u05d9\u05dd \u05dc\u05e2\u05e7\u05d9\u05d1\u05ea-\u05d3\u05d5\u05e4\u05df; \u05d0\u05dd \u05d1\u05de\u05e6\u05d1 follow \u05e2\u05e7\u05d9\u05d1\u05d4 \u05e9\u05dc\u05d0 \u05de\u05ea\u05e7\u05d3\u05de\u05ea \u2014
// \u05de\u05d7\u05dc\u05d9\u05e4\u05d9\u05dd \u05d9\u05d3 \u05db\u05d3\u05d9 \u05dc\u05e9\u05d1\u05d5\u05e8 \u05e1\u05d9\u05de\u05d8\u05e8\u05d9\u05d4 \u05d5\u05dc\u05d4\u05d9\u05de\u05e0\u05e2 \u05de\u05dc\u05d5\u05dc\u05d0\u05d4 \u05d0\u05d9\u05df-\u05e1\u05d5\u05e4\u05d9\u05ea.
function updateStuckDetector() {
  const now = performance.now();
  const d = goalDistance();
  const a = state.nav.progressAnchor;
  if (!a) {
    state.nav.progressAnchor = { dist: d, t: now };
    return;
  }
  if (a.dist - d >= STUCK_PROGRESS_CM) {
    state.nav.progressAnchor = { dist: d, t: now }; // \u05d4\u05ea\u05e7\u05d3\u05de\u05e0\u05d5 \u2014 \u05d0\u05e4\u05e1 \u05e2\u05d5\u05d2\u05df
    return;
  }
  if (now - a.t < STUCK_WINDOW_MS) return;
  // \u05dc\u05d0 \u05d4\u05ea\u05e7\u05d3\u05de\u05e0\u05d5 \u05de\u05e1\u05e4\u05d9\u05e7 \u05d1\u05ea\u05d5\u05da \u05d4\u05d7\u05dc\u05d5\u05df.
  if (state.nav.mode === "seek") {
    enterWallFollow();
  } else if (state.nav.mode === "follow") {
    state.nav.followHand *= -1; // \u05e2\u05e7\u05d9\u05d1\u05d4 \u05ea\u05e7\u05d5\u05e2\u05d4 -> \u05d4\u05d7\u05dc\u05e3 \u05d9\u05d3
    state.nav.wallSince = now;
    state.nav.progressAnchor = { dist: d, t: now };
  }
}

// \u05db\u05e0\u05d9\u05e1\u05d4 \u05dc\u05de\u05e6\u05d1 \u05e2\u05e7\u05d9\u05d1\u05ea-\u05d3\u05d5\u05e4\u05df: \u05d1\u05d5\u05d7\u05e8\u05d9\u05dd \u05d0\u05ea \u05d4\u05e6\u05d3 \u05e9\u05d1\u05d5 \u05d4\u05de\u05db\u05e9\u05d5\u05dc \u05e7\u05e8\u05d5\u05d1 \u05d9\u05d5\u05ea\u05e8 \u05db"\u05d9\u05d3" \u05dc\u05e2\u05e7\u05d9\u05d1\u05d4.
function enterWallFollow() {
  const distLeft = getMemoryDistance(270, 45);
  const distRight = getMemoryDistance(90, 45);
  state.nav.followHand = distLeft < distRight ? -1 : 1; // -1: \u05de\u05db\u05e9\u05d5\u05dc \u05de\u05e9\u05de\u05d0\u05dc, +1: \u05de\u05d9\u05de\u05d9\u05df
  state.nav.mode = "follow";
  state.nav.wallSince = performance.now();
  state.avoidDir = 0;
  state.nav.progressAnchor = { dist: goalDistance(), t: performance.now() };
}

// \u05e2\u05e7\u05d9\u05d1\u05ea \u05e7\u05d5-\u05de\u05ea\u05d0\u05e8 (Bug2). \u05de\u05d7\u05d6\u05d9\u05e8 true \u05db\u05dc \u05e2\u05d5\u05d3 \u05e0\u05e9\u05d0\u05e8\u05d9\u05dd \u05d1\u05e2\u05e7\u05d9\u05d1\u05d4 (\u05d5\u05db\u05d1\u05e8 \u05e0\u05e7\u05d1\u05e2\u05d5
// \u05de\u05d4\u05d9\u05e8\u05d5\u05d9\u05d5\u05ea \u05d4\u05de\u05e0\u05d5\u05e2\u05d9\u05dd), \u05d0\u05d5 false \u05db\u05e9\u05d4\u05d5\u05d7\u05dc\u05d8 \u05dc\u05e2\u05d6\u05d5\u05d1 \u05d5\u05dc\u05d7\u05d6\u05d5\u05e8 \u05dc-seek \u05d1\u05d0\u05d5\u05ea\u05d5 \u05d8\u05d9\u05e7.
function followWall(distFront, goalRel) {
  const hand = state.nav.followHand;
  const sideBearing = hand > 0 ? 90 : 270;
  const sideDist = getMemoryDistance(sideBearing, 45);

  // \u05ea\u05e0\u05d0\u05d9 \u05e2\u05d6\u05d9\u05d1\u05d4 (Bug2): \u05db\u05d9\u05d5\u05d5\u05df \u05d4\u05d9\u05e2\u05d3 \u05e4\u05ea\u05d5\u05d7 \u05e9\u05d5\u05d1 \u05d5\u05d4\u05d7\u05e8\u05d8\u05d5\u05dd \u05e4\u05e0\u05d5\u05d9 -> \u05d7\u05d6\u05d5\u05e8 \u05dc\u05e9\u05d9\u05d5\u05d8 \u05d0\u05dc \u05d4\u05d9\u05e2\u05d3.
  const goalClear = getMemoryDistance(
    clamp(goalRel, -OPEN_SCAN_DEG, OPEN_SCAN_DEG),
    OPEN_SCAN_TOL
  );
  if (
    Math.abs(goalRel) < 70 &&
    goalClear > WALL_LEAVE_CLEAR_CM &&
    distFront > CLEAR_DISTANCE_CM
  ) {
    state.nav.mode = "seek";
    state.nav.progressAnchor = { dist: goalDistance(), t: performance.now() };
    return false;
  }

  // \u05d7\u05e1\u05d9\u05de\u05d4 \u05d7\u05d6\u05d9\u05ea\u05d9\u05ea \u05ea\u05d5\u05da \u05db\u05d3\u05d9 \u05e2\u05e7\u05d9\u05d1\u05d4 -> \u05e4\u05e0\u05d4 \u05d7\u05d3 \u05d4\u05e8\u05d7\u05e7 \u05de\u05d4\u05d3\u05d5\u05e4\u05df.
  if (distFront < SAFE_DISTANCE_CM) {
    applyAvoidTurn(-hand);
    return true;
  }

  // \u05d1\u05e7\u05e8-P \u05dc\u05e9\u05de\u05d9\u05e8\u05ea \u05de\u05e8\u05d7\u05e7-\u05e1\u05e3 \u05de\u05d4\u05d3\u05d5\u05e4\u05df. sideDist \u05d2\u05d3\u05d5\u05dc (\u05d0\u05d9\u05df \u05d3\u05d5\u05e4\u05df) -> \u05e9\u05d2\u05d9\u05d0\u05d4 \u05d7\u05d9\u05d5\u05d1\u05d9\u05ea ->
  // \u05e4\u05d5\u05e0\u05d4 \u05d0\u05dc \u05d4\u05e6\u05d3 \u05dc\u05d7\u05e4\u05e9 \u05d0\u05ea \u05d4\u05d3\u05d5\u05e4\u05df; \u05e7\u05e8\u05d5\u05d1 \u05de\u05d3\u05d9 -> \u05e9\u05d2\u05d9\u05d0\u05d4 \u05e9\u05dc\u05d9\u05dc\u05d9\u05ea -> \u05de\u05ea\u05e8\u05d7\u05e7.
  const error = clamp(sideDist, 0, 400) - WALL_FOLLOW_STANDOFF_CM;
  const steer = clamp(hand * error * WALL_FOLLOW_GAIN, -STEER_MAX, STEER_MAX);
  state.cmd.leftSpeed = clamp(WALL_FOLLOW_SPEED - steer, -255, 255);
  state.cmd.rightSpeed = clamp(WALL_FOLLOW_SPEED + steer, -255, 255);
  return true;
}

function updateAutonomousCommand() {
  if (!state.nav.goal) initNavGoal();

  // הגענו לקצה השני של הבריכה -> עצירה מלאה.
  if (goalDistance() <= GOAL_ARRIVE_CM) {
    state.nav.mode = "arrived";
    state.avoidDir = 0;
    state.cmd.leftSpeed = 0;
    state.cmd.rightSpeed = 0;
    return;
  }

  const distFront = getMemoryDistance(0, 45);
  const distLeft = getMemoryDistance(270, 45);
  const distRight = getMemoryDistance(90, 45);
  const goalRel = goalBearingRel();

  // גלאי-תקיעה: מנטר התקדמות נטו אל היעד; עשוי להעביר ל"follow" (עקיבת-דופן).
  updateStuckDetector();

  // מצב עקיבת-דופן (Bug2): מחלץ ממלכודות קעורות/פינות. אם החליט לעזוב הוא מחזיר
  // false ונופלים למטה למצב seek באותו טיק.
  if (state.nav.mode === "follow") {
    if (followWall(distFront, goalRel)) return;
  }

  // --- מצב seek: follow-the-gap מוטה-יעד + התחמקות מיידית ---
  // התחמקות מחויבת: החרטום קרוב מדי -> מסתובב במקום לצד הפתוח עד שהחזית נפתחת
  // (hysteresis), כדי למנוע ריצוד שמאל/ימין כשהצדדים כמעט שווים.
  if (state.avoidDir !== 0) {
    if (distFront >= CLEAR_DISTANCE_CM) {
      state.avoidDir = 0;
    } else {
      applyAvoidTurn(state.avoidDir);
      return;
    }
  }

  if (distFront < SAFE_DISTANCE_CM) {
    state.avoidDir = distLeft > distRight ? -1 : 1;
    applyAvoidTurn(state.avoidDir);
    return;
  }

  // שיוט: בוחר את הכיוון הפתוח ביותר בקשת החזית, מוטה אל אזימוט היעד. כך הסירה
  // נמשכת אל הקצה השני תוך התחמקות מגופים, במקום לשוטט למים פתוחים סתם.
  const targetBearing = chooseOpenHeading(
    clamp(goalRel, -OPEN_SCAN_DEG, OPEN_SCAN_DEG),
    GOAL_ATTRACT_PENALTY
  );
  const steer = clamp(targetBearing * OPEN_STEER_GAIN, -STEER_MAX, STEER_MAX);

  const fwdClear = getMemoryDistance(0, 25);
  const openness = clamp(
    (fwdClear - SAFE_DISTANCE_CM) / (STEER_LOOKAHEAD_CM - SAFE_DISTANCE_CM),
    0,
    1
  );
  const speed = Math.round(
    AUTONOMOUS_MIN_SPEED + openness * (AUTONOMOUS_SPEED - AUTONOMOUS_MIN_SPEED)
  );

  // steer > 0 raises the right motor above the left => turns right.
  state.cmd.leftSpeed = clamp(speed - steer, -255, 255);
  state.cmd.rightSpeed = clamp(speed + steer, -255, 255);
}

function updateCommandUI() {
  leftSpeedValue.textContent = String(state.cmd.leftSpeed);
  rightSpeedValue.textContent = String(state.cmd.rightSpeed);
}

// Safety envelope: intercepts outgoing steering commands (manual or autonomous)
// right before they're sent. Scans every accumulated map point and every live
// radar reading for the nearest obstacle; if it breaches the warning/safety
// radius, returns a COPY of the command with a repulsive steering response.
// The base state.cmd is never mutated, so the moment the boat leaves the orange
// zone control returns cleanly to the autonomous navigator / manual joystick.
function computeSafeCommand() {
  const cmd = {
    leftSpeed: state.cmd.leftSpeed,
    rightSpeed: state.cmd.rightSpeed,
    winchSpeed: state.cmd.winchSpeed,
    radarAngle: state.cmd.radarAngle,
  };

  const bx = state.pose.x;
  const by = state.pose.y;
  const heading = state.pose.headingDeg;

  let minDist = Infinity;
  let leftThreat = 0;
  let rightThreat = 0;
  let nearestRelBearing = 0; // הזווית היחסית של המכשול הקרוב ביותר (0 = מלפנים, ±180 = מאחור)
  // המרחק הפנוי הקרוב ביותר בחרוט שלפני החרטום — מזין את מושל המהירות. נבדק על
  // טווח רחב יותר מטווח האזהרה, כדי שהסירה תתחיל להאט מבעוד מועד.
  let forwardClearance = Infinity;

  // פונקציית עזר להערכת רמת איום של נקודה במרחב
  const evaluateThreat = (px, py, confidence) => {
    const dist = Math.hypot(px - bx, py - by);

    // חישוב הזווית היחסית לחרטום הסירה
    const absBearing = (Math.atan2(px - bx, py - by) * 180) / Math.PI;
    let relBearing = ((absBearing - heading + 540) % 360) - 180;

    // מרחק פנוי בחרוט הקדמי — למושל המהירות (טווח רחב יותר מהתחמקות).
    if (
      dist <= GOVERNOR_RANGE_CM &&
      Math.abs(relBearing) < GOVERNOR_FWD_CONE_DEG &&
      dist < forwardClearance
    ) {
      forwardClearance = dist;
    }

    if (dist > AVOID_WARNING_RADIUS) return;

    // שמירת הזווית של המכשול הקרוב ביותר, כדי שהבריחה תדע לאיזה כיוון להימלט
    if (dist < minDist) {
      minDist = dist;
      nearestRelBearing = relBearing;
    }

    // רלוונטי רק לעצמים שנמצאים מולנו (100 מעלות לכל כיוון מהחרטום)
    if (Math.abs(relBearing) < 100) {
      // איום גדל ככל שהמכשול קרוב יותר לקו האדום, ומוכפל ברמת הביטחון (Solidness)
      const threatLevel = (AVOID_WARNING_RADIUS - dist) * confidence;

      if (relBearing > 0) {
        rightThreat += threatLevel; // המכשול מימין
      } else {
        leftThreat += threatLevel;  // המכשול משמאל
      }
    }
  };

  // 1. סריקת המפה המצטברת — רק תאים שבהם מספר קשתות נחתכו (לא רפאים)
  for (const cell of mapCells.values()) {
    const confidence = getCellConfidence(cell);
    if (confidence >= MAP_CONFIRM_HITS) {
      evaluateThreat(cell.x, cell.y, confidence);
    }
  }

  // 2. סריקת המכ"ם החי (מקבל משקל גבוה כי אלו נתונים טריים)
  const now = performance.now();
  for (const [absSlot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999 || (now - entry.t > RADAR_TTL_MS)) continue;
    const rad = degToRad(absSlot);
    const px = bx + Math.sin(rad) * entry.value;
    const py = by + Math.cos(rad) * entry.value;
    evaluateThreat(px, py, 5); // משקל של 5 למידע חי
  }

  // הכל פנוי — גם החזית פתוחה לגמרי.
  if (minDist > AVOID_WARNING_RADIUS && forwardClearance >= GOVERNOR_RANGE_CM) {
    return cmd;
  }

  // "קדימה" נמדד לפי המהירות הנטו (סכום המנועים), לא לפי מנוע בודד חיובי.
  // סיבוב במקום (למשל 170,-170 של התחמקות אוטונומית) הוא בעל מהירות נטו 0 —
  // הוא אינו מקדם את הסירה לעבר המכשול, ולכן אסור להתייחס אליו כפקודת התקדמות.
  // אחרת שכבת הבטיחות מייצרת היגוי-נגדי שמבטל את הסיבוב, והסירה נתקעת ומסתובבת
  // כמעט במקום מול קיר לנצח (deadlock).
  const netForward = cmd.leftSpeed + cmd.rightSpeed;
  const isForwardCommand = netForward > 20;

  if (minDist <= AVOID_SAFETY_RADIUS) {
    // --- חציית קו אדום: התנגשות מיידית ---
    if (isForwardCommand) {
      // ביצוע J-Turn: נסיעה לאחור תוך הפניית החרטום לכיוון הפנוי.
      // בסימולציה turn = (right - left): heading גדל (פנייה ימינה) כש-right > left.
      if (rightThreat > leftThreat) {
        // סכנה מימין -> החרטום צריך לפנות שמאלה (heading יורד => right < left)
        cmd.leftSpeed = -80;
        cmd.rightSpeed = -180;
      } else {
        // סכנה משמאל -> החרטום צריך לפנות ימינה (heading עולה => right > left)
        cmd.leftSpeed = -180;
        cmd.rightSpeed = -80;
      }
    } else if (Math.abs(nearestRelBearing) > 90) {
      // אם הסירה נוסעת רוורס לתוך משהו שמאחור, בריחה חזקה קדימה.
      // רק כשהמכשול הקרוב ביותר באמת מאחור — אחרת דחיפה קדימה רק תיסע לתוכו.
      cmd.leftSpeed = 150;
      cmd.rightSpeed = 150;
    }
    // מכשול קרוב מלפנים והפקודה אינה קדימה: הסירה כבר מתרחקת (או עומדת),
    // אין צורך בבריחה — משאירים את הפקודה המקורית ולא דוחפים לתוך המכשול.
  } else if (minDist <= AVOID_WARNING_RADIUS && isForwardCommand) {
    // --- כניסה לטווח הכתום: התחמקות אקטיבית ---
    const evasionForce = 1 - ((minDist - AVOID_SAFETY_RADIUS) / (AVOID_WARNING_RADIUS - AVOID_SAFETY_RADIUS));
    const turnSpeed = Math.round(evasionForce * 255);

    // היגוי נגדי על בסיס *סך כל האיומים* ולא רק הנקודה הקרובה ביותר.
    // פנייה שמאלה = מנוע ימין מהיר יותר; פנייה ימינה = מנוע שמאל מהיר יותר.
    if (rightThreat > leftThreat) {
      // אגף ימין חסום יותר -> שבור שמאלה
      cmd.leftSpeed = clamp(cmd.leftSpeed - turnSpeed, -255, 255);
      cmd.rightSpeed = clamp(cmd.rightSpeed + turnSpeed, -255, 255);
    } else {
      // אגף שמאל חסום יותר -> שבור ימינה
      cmd.leftSpeed = clamp(cmd.leftSpeed + turnSpeed, -255, 255);
      cmd.rightSpeed = clamp(cmd.rightSpeed - turnSpeed, -255, 255);
    }
  }

  // --- מושל מהירות קדימה (אנטי-נגיחה בלבד) ---
  // חיתוך אחרון וקשיח: מהירות ההתקדמות *נטו* (סכום המנועים) מוגבלת פרופורציונלית
  // למרחק הפנוי בחרוט הקדמי, כך שהסירה לעולם לא דוהרת לתוך גוף שזוהה — היא עוצרת
  // בהתקדמות במרחק GOVERNOR_STOP_CM. החיתוך מוריד ערך שווה משני המנועים ולכן שומר
  // על הפרש המנועים (הסיבוב/היגוי) — הסירה עדיין יכולה להסתובב ולפנות, רק לא
  // להתקדם לתוך המכשול. כל ההיגוי (חיפוש מרחב פתוח + סיבוב-בריחה) מנוהל ע"י שכבת
  // הניווט; המושל אינו כופה כיוון פנייה משלו, כדי לא להיאבק עם הניווט ולתקוע את
  // הסירה. תנועה לאחור אינה מוגבלת (בריחה). פועל גם בשליטה ידנית וגם באוטונומית.
  const netOut = cmd.leftSpeed + cmd.rightSpeed;
  if (netOut > 0 && forwardClearance < GOVERNOR_RANGE_CM) {
    const frac = clamp(
      (forwardClearance - GOVERNOR_STOP_CM) / (GOVERNOR_RANGE_CM - GOVERNOR_STOP_CM),
      0,
      1
    );
    const cap = frac * 510; // מהירות נטו מירבית מותרת (שני מנועים על 255)
    if (netOut > cap) {
      const reduce = (netOut - cap) / 2;
      cmd.leftSpeed = clamp(cmd.leftSpeed - reduce, -255, 255);
      cmd.rightSpeed = clamp(cmd.rightSpeed - reduce, -255, 255);
    }
  }

  return cmd;
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

// Top-down "ground truth" of the mock world: boundary walls, obstacle bodies,
// the boat at its real pose, and the circle marking the ultrasonic range. Lets
// the operator compare the actual world against the radar reconstruction.
function drawWorld() {
  const w = worldCanvas.width;
  const h = worldCanvas.height;

  worldCtx.clearRect(0, 0, w, h);
  worldCtx.fillStyle = "rgba(3, 20, 30, 0.95)";
  worldCtx.fillRect(0, 0, w, h);

  if (!world.loaded || !state.mockEnabled) {
    worldCtx.fillStyle = "rgba(150, 200, 220, 0.7)";
    worldCtx.font = "16px system-ui, sans-serif";
    worldCtx.textAlign = "center";
    worldCtx.fillText("הפעל מצב מוק דאטא לצפייה בעולם", w / 2, h / 2);
    worldPosValue.textContent = "--";
    worldInRangeValue.textContent = "--";
    requestAnimationFrame(drawWorld);
    return;
  }

  // Fit the whole world rectangle into the canvas (with a small margin) and
  // keep the boat centred so it never scrolls out of view.
  const margin = 16;
  const bx = state.telemetry.boatX ?? 0;
  const by = state.telemetry.boatY ?? 0;
  const heading = state.telemetry.boatHeadingDeg ?? 0;
  const scale = Math.min(
    (w - 2 * margin) / (2 * world.boundsHalfX),
    (h - 2 * margin) / (2 * world.boundsHalfY)
  );
  const originX = w / 2;
  const originY = h / 2;
  // World (wx, wy) -> screen. World +y is north (up); canvas y grows down.
  const toScreen = (wx, wy) => ({
    x: originX + (wx - bx) * scale,
    y: originY - (wy - by) * scale,
  });

  // Boundary walls
  const tl = toScreen(-world.boundsHalfX, world.boundsHalfY);
  const br = toScreen(world.boundsHalfX, -world.boundsHalfY);
  worldCtx.strokeStyle = "rgba(120, 170, 200, 0.8)";
  worldCtx.lineWidth = 2;
  worldCtx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

  const boat = toScreen(bx, by);
  const rangePx = world.maxRange * scale;

  // Radar range circle
  worldCtx.strokeStyle = "rgba(44, 255, 197, 0.55)";
  worldCtx.setLineDash([6, 6]);
  worldCtx.lineWidth = 1.5;
  worldCtx.beginPath();
  worldCtx.arc(boat.x, boat.y, rangePx, 0, Math.PI * 2);
  worldCtx.stroke();
  worldCtx.setLineDash([]);

  // Obstacle bodies; highlight the ones the sensors could actually reach.
  let inRange = 0;
  for (const t of world.targets) {
    const p = toScreen(t.x, t.y);
    const dist = Math.hypot(t.x - bx, t.y - by);
    const within = dist - t.radius <= world.maxRange;
    if (within) inRange += 1;
    worldCtx.beginPath();
    worldCtx.arc(p.x, p.y, Math.max(2, t.radius * scale), 0, Math.PI * 2);
    worldCtx.fillStyle = within ? "rgba(44, 255, 197, 0.85)" : "rgba(120, 140, 150, 0.5)";
    worldCtx.fill();
  }

  // Sensor beam directions in the world frame (heading + servo sweep + offset).
  const sweep = state.telemetry.radarAngle ?? 0;
  worldCtx.strokeStyle = "rgba(255, 241, 118, 0.35)";
  worldCtx.lineWidth = 1;
  for (const beam of SENSOR_BEAMS) {
    const rad = degToRad(heading + sweep + beam.dir);
    worldCtx.beginPath();
    worldCtx.moveTo(boat.x, boat.y);
    worldCtx.lineTo(boat.x + Math.sin(rad) * rangePx, boat.y - Math.cos(rad) * rangePx);
    worldCtx.stroke();
  }

  // Boat, rotated to its real heading (0 = north / up, clockwise positive).
  worldCtx.save();
  worldCtx.translate(boat.x, boat.y);
  worldCtx.rotate(degToRad(heading));
  worldCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  worldCtx.beginPath();
  worldCtx.moveTo(0, -12);
  worldCtx.lineTo(8, 9);
  worldCtx.lineTo(0, 5);
  worldCtx.lineTo(-8, 9);
  worldCtx.closePath();
  worldCtx.fill();
  worldCtx.restore();

  worldPosValue.textContent = `${Math.round(bx)}, ${Math.round(by)} · ${Math.round(heading)}°`;
  worldInRangeValue.textContent = String(inRange);

  requestAnimationFrame(drawWorld);
}

// Fold each sensor reading into the accumulated grid map as an ARC (the beam is
// ~15° wide, so the true obstacle can lie anywhere along that arc at the
// measured range). Also clears the empty cone in front of the arc and records
// the boat's path (trail). Cells only become "real" where arcs intersect.
function accumulateRadarMap() {
  const sweep = state.telemetry.radarAngle ?? 0;
  const heading = state.pose.headingDeg;
  const bx = state.pose.x;
  const by = state.pose.y;

  // Trail: append when the boat has moved a little, to keep the array bounded.
  const last = boatTrail[boatTrail.length - 1];
  if (!last || Math.hypot(last.x - bx, last.y - by) > 3) {
    boatTrail.push({ x: bx, y: by });
    if (boatTrail.length > MAP_TRAIL_MAX) boatTrail.shift();
  }

  const boatKx = Math.round(bx / MAP_CELL_CM);
  const boatKy = Math.round(by / MAP_CELL_CM);

  for (const beam of SENSOR_BEAMS) {
    const dist = state.telemetry[beam.key];
    if (dist == null) continue;

    // Skip a duplicate look (same beam, servo step, boat cell and range) so a
    // stationary boat can't pump a lone arc past the intersection threshold.
    const distBucket = dist >= 999 ? 999 : Math.round(dist / MAP_CELL_CM);
    const sig = `${sweep}:${boatKx}:${boatKy}:${distBucket}`;
    if (lastArcObs.get(beam.key) === sig) continue;
    lastArcObs.set(beam.key, sig);

    // Absolute world bearing of the beam centre = heading + servo sweep + offset.
    const center = heading + sweep + beam.dir;
    const clearTo = dist >= 999 ? sim.maxRange : dist;

    // Empty cone: every ray across the beam width, up to just before the arc, is
    // confirmed free water. Collect the cells once (a Set) so overlapping rays
    // don't decrement the same cell several times in a single frame.
    const clearKeys = new Set();
    for (let a = -MAP_ARC_FOV_DEG; a <= MAP_ARC_FOV_DEG; a += MAP_ARC_STEP_DEG) {
      collectRayCells(bx, by, degToRad(center + a), clearTo, clearKeys);
    }
    for (const key of clearKeys) {
      const cell = mapCells.get(key);
      if (cell) {
        cell.hits -= MAP_CLEAR_DECAY;
        if (cell.hits <= 0) mapCells.delete(key);
      }
    }

    // A 999 reading only clears space — there is no obstacle arc to record.
    if (dist >= 999) continue;

    // Occupied arc: paint every cell the arc passes through, but only ONCE per
    // arc (a Set), so a single arc contributes at most its (range-scaled) weight
    // to each cell and thus stays below MAP_CONFIRM_HITS until another arc
    // crosses it. Far arcs weigh less than near ones (see arcCellWeight).
    const weight = arcCellWeight(dist);
    const arcKeys = new Set();
    for (let a = -MAP_ARC_FOV_DEG; a <= MAP_ARC_FOV_DEG; a += MAP_ARC_STEP_DEG) {
      const rad = degToRad(center + a);
      const wx = bx + Math.sin(rad) * dist;
      const wy = by + Math.cos(rad) * dist;
      const kx = Math.round(wx / MAP_CELL_CM);
      const ky = Math.round(wy / MAP_CELL_CM);
      const key = `${kx},${ky}`;
      if (arcKeys.has(key)) continue;
      arcKeys.add(key);
      markArcCell(wx, wy, kx, ky, key, weight);
    }
  }
}

// Weight = MAP_ARC_REF_CM / range, so a reading at MAP_ARC_REF_CM scores 1.0,
// nearer readings score MORE (a tight, well-localised beam earns extra trust and
// can confirm on its own), and farther readings score LESS. Capped at
// MAP_ARC_MAX_WEIGHT so a near arc confirms in one look, floored so distant arcs
// still smear thinly and must intersect before a lone far echo is believed.
function arcCellWeight(dist) {
  return Math.min(MAP_ARC_MAX_WEIGHT, MAP_ARC_REF_CM / Math.max(dist, 1));
}

// Add a single arc's contribution to one grid cell. Each arc adds at most its
// range-scaled `weight`. A weighted running average keeps the stored point near
// the true intersecting surface, and total weight is capped so ray-clearing can
// still erode a cell that goes empty.
function markArcCell(wx, wy, kx, ky, key, weight) {
  const cell = mapCells.get(key);
  if (cell) {
    const newHits = Math.min(MAP_HIT_CAP, cell.hits + weight);
    const added = newHits - cell.hits;
    if (added > 0) {
      cell.x += (wx - cell.x) * (added / newHits);
      cell.y += (wy - cell.y) * (added / newHits);
      cell.hits = newHits;
    }
  } else if (mapCells.size < MAP_MAX_CELLS) {
    mapCells.set(key, { x: wx, y: wy, hits: weight, kx, ky });
  }
}

// Ray clearing (inverse sensor model): the straight line between the boat and a
// hit point must be empty water, otherwise the beam would have bounced sooner.
// Collect every cell along that free segment into `out`; a margin of one cell
// before the hit is left out so the real surface is never eroded by its own arc.
function collectRayCells(bx, by, rad, clearDist, out) {
  const sinR = Math.sin(rad);
  const cosR = Math.cos(rad);
  const end = clearDist - MAP_CELL_CM;
  for (let t = MAP_CELL_CM; t < end; t += MAP_CELL_CM) {
    const kx = Math.round((bx + sinR * t) / MAP_CELL_CM);
    const ky = Math.round((by + cosR * t) / MAP_CELL_CM);
    out.add(`${kx},${ky}`);
  }
}

function clearRadarMap() {
  mapCells.clear();
  boatTrail.length = 0;
  lastArcObs.clear();
}

// Arc-intersection confidence: with arc mapping each DISTINCT arc adds at most
// +1 to a cell, so cell.hits is literally the number of independent arcs that
// crossed it. A lone arc leaves 1; a genuine obstacle where arcs intersect
// climbs to >= MAP_CONFIRM_HITS. Returning the raw hit count keeps that meaning
// intact (a neighbourhood sum would wrongly boost a single arc's straight tail).
function getCellConfidence(cell) {
  return cell.hits;
}

// World-fixed reconstruction: draws the accumulated grid, the boat's trail and
// its live pose, auto-fitting the view to everything collected so far. North is
// up (unlike the boat-centric radar view, this frame does NOT rotate).
function drawMap() {
  const w = mapCanvas.width;
  const h = mapCanvas.height;

  mapCtx.clearRect(0, 0, w, h);
  mapCtx.fillStyle = "rgba(3, 16, 24, 0.95)";
  mapCtx.fillRect(0, 0, w, h);

  mapPointsValue.textContent = String(mapCells.size);

  if (mapCells.size === 0) {
    mapCtx.fillStyle = "rgba(150, 200, 220, 0.7)";
    mapCtx.font = "16px system-ui, sans-serif";
    mapCtx.textAlign = "center";
    mapCtx.fillText('נוע עם הסירה כדי לבנות מפה מנתוני המכ"ם', w / 2, h / 2);
    requestAnimationFrame(drawMap);
    return;
  }

  const bx = state.pose.x;
  const by = state.pose.y;

  // Fit bounds around all cells, the trail and the boat.
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  const consider = (x, y) => {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  };
  for (const c of mapCells.values()) consider(c.x, c.y);
  for (const p of boatTrail) consider(p.x, p.y);
  consider(bx, by);

  const margin = 22;
  const minSpan = 200;
  const spanX = Math.max(maxX - minX, minSpan);
  const spanY = Math.max(maxY - minY, minSpan);
  const cxWorld = (minX + maxX) / 2;
  const cyWorld = (minY + maxY) / 2;
  const scale = Math.min((w - 2 * margin) / spanX, (h - 2 * margin) / spanY);
  const originX = w / 2;
  const originY = h / 2;
  // World +y is north (up); canvas y grows down.
  const toScreen = (wx, wy) => ({
    x: originX + (wx - cxWorld) * scale,
    y: originY - (wy - cyWorld) * scale,
  });

  // Faint 100 cm grid for scale reference.
  mapCtx.strokeStyle = "rgba(80, 130, 150, 0.18)";
  mapCtx.lineWidth = 1;
  const gridCm = 100;
  const gx0 = Math.floor((cxWorld - spanX / 2) / gridCm) * gridCm;
  const gx1 = Math.ceil((cxWorld + spanX / 2) / gridCm) * gridCm;
  for (let gx = gx0; gx <= gx1; gx += gridCm) {
    const a = toScreen(gx, cyWorld - spanY);
    const b = toScreen(gx, cyWorld + spanY);
    mapCtx.beginPath();
    mapCtx.moveTo(a.x, a.y);
    mapCtx.lineTo(b.x, b.y);
    mapCtx.stroke();
  }
  const gy0 = Math.floor((cyWorld - spanY / 2) / gridCm) * gridCm;
  const gy1 = Math.ceil((cyWorld + spanY / 2) / gridCm) * gridCm;
  for (let gy = gy0; gy <= gy1; gy += gridCm) {
    const a = toScreen(cxWorld - spanX, gy);
    const b = toScreen(cxWorld + spanX, gy);
    mapCtx.beginPath();
    mapCtx.moveTo(a.x, a.y);
    mapCtx.lineTo(b.x, b.y);
    mapCtx.stroke();
  }

  // Boat trail.
  if (boatTrail.length > 1) {
    mapCtx.strokeStyle = "rgba(255, 241, 118, 0.45)";
    mapCtx.lineWidth = 1.5;
    mapCtx.beginPath();
    boatTrail.forEach((p, i) => {
      const s = toScreen(p.x, p.y);
      if (i === 0) mapCtx.moveTo(s.x, s.y);
      else mapCtx.lineTo(s.x, s.y);
    });
    mapCtx.stroke();
  }

  // Accumulated detections; brighter = more arcs intersect there. Lone-arc
  // cells (below MAP_CONFIRM_HITS) are drawn as faint grey "ghosts" so the
  // operator still sees them but knows the navigation logic is ignoring them.
  for (const c of mapCells.values()) {
    const s = toScreen(c.x, c.y);
    const confidence = getCellConfidence(c);
    const isGhost = confidence < MAP_CONFIRM_HITS;
    const alpha = isGhost ? 0.1 : Math.min(1, 0.3 + confidence * 0.1);
    mapCtx.fillStyle = isGhost
      ? `rgba(150, 150, 150, ${alpha})`
      : `rgba(44, 255, 197, ${alpha})`;
    const size = Math.max(2, MAP_CELL_CM * scale * 0.5);
    mapCtx.beginPath();
    mapCtx.arc(s.x, s.y, size, 0, Math.PI * 2);
    mapCtx.fill();
  }

  // Boat at its live pose, rotated to heading (0 = north / up).
  const boat = toScreen(bx, by);
  mapCtx.save();
  mapCtx.translate(boat.x, boat.y);
  mapCtx.rotate(degToRad(state.pose.headingDeg));
  mapCtx.fillStyle = "rgba(255, 241, 118, 0.95)";
  mapCtx.beginPath();
  mapCtx.moveTo(0, -12);
  mapCtx.lineTo(8, 9);
  mapCtx.lineTo(0, 5);
  mapCtx.lineTo(-8, 9);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.restore();

  // שרטוט טווחי בטיחות והתחמקות סביב הסירה
  const sBoat = toScreen(bx, by);

  // מעגל פנייה/הדיפה (כתום מקווקו)
  mapCtx.strokeStyle = "rgba(255, 165, 0, 0.4)";
  mapCtx.setLineDash([4, 4]);
  mapCtx.lineWidth = 1.5;
  mapCtx.beginPath();
  mapCtx.arc(sBoat.x, sBoat.y, AVOID_WARNING_RADIUS * scale, 0, Math.PI * 2);
  mapCtx.stroke();
  mapCtx.setLineDash([]);

  // מעגל סכנה (אדום)
  mapCtx.strokeStyle = "rgba(255, 100, 100, 0.4)";
  mapCtx.beginPath();
  mapCtx.arc(sBoat.x, sBoat.y, AVOID_SAFETY_RADIUS * scale, 0, Math.PI * 2);
  mapCtx.stroke();

  requestAnimationFrame(drawMap);
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
  // The whole sensor array is rotated by the current servo angle AND by the
  // boat's heading. Store each reading at its ABSOLUTE world bearing so that a
  // rotating boat leaves the world fixed in place (the radar view then rotates
  // the world back around the boat at draw time, keeping the bow pointing up).
  const sweep = state.telemetry.radarAngle ?? 0;
  const heading = state.pose.headingDeg;
  const now = performance.now();
  for (const beam of SENSOR_BEAMS) {
    const absSlot = normalizeDeg(heading + beam.dir + sweep);
    const newVal = state.telemetry[beam.key];
    if (newVal == null) continue;
    // Always refresh the timestamp when this slot is actually measured, so a
    // continuously confirmed obstacle stays alive; keep the smoothed value to
    // avoid flicker from minor (<5 cm) sensor noise.
    const existing = radarMemory.get(absSlot);
    const value =
      existing && Math.abs(newVal - existing.value) <= 5 ? existing.value : newVal;
    radarMemory.set(absSlot, { value, t: now });
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

  // radarMemory holds ABSOLUTE world bearings. Subtract the current heading so
  // the world rotates around the boat and the bow stays pointing up on screen.
  const heading = state.pose.headingDeg;

  // Collect the currently valid detections as points at their true
  // (bearing, distance) location, so a straight wall lands on a straight line
  // and scattered bodies stay separate dots.
  const points = [];
  for (const [absSlot, entry] of radarMemory) {
    if (!entry.value || entry.value >= 999) continue;
    const pixelDist = (entry.value / sim.maxRange) * maxR;
    const relSlot = normalizeDeg(absSlot - heading);
    const rad = degToRad(relSlot) - Math.PI / 2;
    points.push({
      slot: absSlot,
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

