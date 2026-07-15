// Offline nav replay harness.
// Loads the REAL reactive navigator from web/app.js (no reimplementation) inside
// a stubbed browser sandbox, then feeds recorded real-hardware telemetry
// (host/logs/nav-*.ndjson) through it frame-by-frame and scores the decisions
// with simple heuristics. Open-loop: the world can't respond to new commands, so
// this validates PERCEPTION + DECISION SANITY + BOW CALIBRATION on real sensor
// noise, not closed-loop trajectory success.

const fs = require("fs");
const path = require("path");
const vm = require("vm");

const APP = path.join(__dirname, "web", "app.js");
const LOG_DIR = path.join(__dirname, "logs");

// ---- Virtual clock (drives performance.now so TTL/pulse timers are deterministic) ----
let VCLOCK = 0;

// ---- Permissive callable stub for all browser globals we don't care about ----
const stub = new Proxy(function () {}, {
  get(_t, prop) {
    if (prop === "then") return undefined; // never look thenable to await
    if (prop === Symbol.toPrimitive) return () => 0;
    if (prop === "length") return 0;
    return stub;
  },
  apply() {
    return stub;
  },
  construct() {
    return stub;
  },
  set() {
    return true;
  },
});

function loadNav() {
  let src = fs.readFileSync(APP, "utf8");
  // Export the internals we need (captured from the same lexical scope).
  src +=
    "\n;globalThis.__NAV__ = { state, liveScan, rawPrev, SENSOR_BEAMS," +
    " updateLiveScan, liveCone, liveDistance, bestOpenBearing," +
    " updateAutonomousRealworld, realworldSafeCommand, shapeMotorSpeed," +
    " resetBowEst: function(){ bowEstSin = 0; bowEstCos = 0; bowEstWeight = 0; bowCandidateDeg = null; bowStableHits = 0; } };\n";

  const sandbox = {
    document: stub,
    window: stub,
    navigator: stub,
    location: stub,
    localStorage: stub,
    EventSource: stub,
    fetch: () => stub,
    setInterval: () => 0,
    setTimeout: () => 0,
    clearInterval: () => {},
    clearTimeout: () => {},
    requestAnimationFrame: () => 0,
    cancelAnimationFrame: () => {},
    performance: { now: () => VCLOCK },
    console,
    Math,
    Date,
    JSON,
    // common DOM-ish globals referenced at top level
    connectBtn: stub,
    modeSwitch: stub,
    refreshPortsBtn: stub,
    mockSwitch: stub,
    mapClearBtn: stub,
  };
  sandbox.globalThis = sandbox;
  sandbox.self = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(src, sandbox, { filename: "app.js" });
  return sandbox.__NAV__;
}

const NAV = loadNav();

// ---- Reset the navigator to a clean autonomy-arm state ----
function resetNav() {
  NAV.liveScan.clear();
  NAV.rawPrev.clear();
  NAV.resetBowEst();
  const s = NAV.state;
  s.mockEnabled = false;
  s.manualMode = false;
  s.avoidDir = 0;
  s.nav.bowOffsetDeg = 60;
  s.nav.bowLocked = false;
  s.nav.rwPhase = "calib";
  s.nav.calibStart = 0;
  s.nav.spinUntil = 0;
  s.nav.settleUntil = 0;
  s.nav.reverseUntil = 0;
  s.cmd.leftSpeed = 0;
  s.cmd.rightSpeed = 0;
  s.cmd.radarAngle = 0;
}

// Heuristic thresholds for judging decisions (mirror the nav's own semantics).
const NEAR_CM = 40; // an obstacle this close on the bow is "near"
const OPEN_CM = 90; // beyond this everything reads open
const SPIN_NET = 20; // |L+R| below this with |L-R| large == spinning/holding (no translation)

function classifyCmd(L, R) {
  const net = L + R;
  const diff = L - R;
  if (Math.abs(net) < SPIN_NET && Math.abs(diff) > 40) return "spin";
  if (net > SPIN_NET) return "forward";
  if (net < -SPIN_NET) return "reverse";
  return "hold";
}

function replayFile(file) {
  const lines = fs.readFileSync(path.join(LOG_DIR, file), "utf8").trim().split("\n").filter(Boolean);
  if (lines.length === 0) return null;
  resetNav();
  const s = NAV.state;

  const m = {
    file,
    frames: 0,
    bowLockedFrame: -1,
    bowFinal: null,
    bowJumps: 0, // large frame-to-frame bow offset changes after lock (instability)
    forwardIntoNear: 0, // net-forward while a fresh near obstacle sits dead ahead (UNSAFE)
    frozenInOpen: 0, // holding/spinning while front+sides all open (LIVENESS loss)
    spinFrames: 0,
    reverseFrames: 0,
    forwardFrames: 0,
    holdFrames: 0,
    staleFrames: 0, // no fresh front data
    avoidFlips: 0, // avoidDir sign changes (chatter)
    steerSignFlips: 0, // steer sign changes frame-to-frame (chatter)
    wrongWayPivot: 0, // committed a pivot toward the MORE BLOCKED side (drives into wall)
    dirFlips: 0, // forward<->reverse net-translation reversals (pointless back-and-forth)
    reverseRuns: 0, // number of distinct reverse episodes
  };

  let prevAvoid = 0;
  let prevSteerSign = 0;
  let prevBow = null;
  let prevNetDir = 0; // +1 last forward, -1 last reverse (spin/hold ignored)
  let inReverse = false;

  for (const line of lines) {
    let rec;
    try {
      rec = JSON.parse(line);
    } catch {
      continue;
    }
    if (!rec.tel) continue;
    VCLOCK = rec.t; // drive the nav clock from the recorded timestamp

    // Feed RAW telemetry exactly as the boat reported it.
    s.telemetry.usFront = rec.tel.usFront;
    s.telemetry.usRight = rec.tel.usRight;
    s.telemetry.usRadar = rec.tel.usRadar;
    s.telemetry.usLeft = rec.tel.usLeft;
    s.telemetry.radarAngle = rec.tel.radarAngle;
    s.telemetry.boatHeadingDeg = rec.tel.boatHeadingDeg ?? 0;

    // Perception update (fills liveScan + runs the online bow estimator using the
    // command still in state.cmd from the previous tick — exactly as live).
    NAV.updateLiveScan();

    if (s.nav.bowLocked && m.bowLockedFrame < 0) m.bowLockedFrame = m.frames;
    if (s.nav.bowLocked && prevBow != null) {
      let d = Math.abs(((s.nav.bowOffsetDeg - prevBow + 540) % 360) - 180);
      if (d > 25) m.bowJumps++;
    }
    prevBow = s.nav.bowOffsetDeg;

    // Snapshot perception the decision will use.
    const fc = NAV.liveCone(0, 22);
    const leftC = NAV.liveCone(-55, 40);
    const rightC = NAV.liveCone(55, 40);
    const frontFresh = fc.count > 0;
    const front = fc.median;

    // Decide.
    const avoidBefore = s.avoidDir;
    NAV.updateAutonomousRealworld();
    NAV.realworldSafeCommand(s.cmd);
    const L = s.cmd.leftSpeed;
    const R = s.cmd.rightSpeed;
    const kind = classifyCmd(L, R);

    // Wrong-way pivot: a freshly committed in-place pivot (avoidDir 0 -> +/-1)
    // that turns the bow toward the MORE BLOCKED side, i.e. into the wall.
    if (avoidBefore === 0 && s.avoidDir !== 0 && leftC.count > 0 && rightC.count > 0) {
      const towardRight = s.avoidDir > 0;
      const rightMoreBlocked = rightC.median < leftC.median - 15;
      const leftMoreBlocked = leftC.median < rightC.median - 15;
      if ((towardRight && rightMoreBlocked) || (!towardRight && leftMoreBlocked))
        m.wrongWayPivot++;
    }

    if (kind === "spin") m.spinFrames++;
    else if (kind === "reverse") m.reverseFrames++;
    else if (kind === "forward") m.forwardFrames++;
    else m.holdFrames++;

    // Pointless back-and-forth: count forward<->reverse net-translation reversals
    // and distinct reverse episodes.
    if (kind === "forward" || kind === "reverse") {
      const nd = kind === "forward" ? 1 : -1;
      if (prevNetDir !== 0 && nd !== prevNetDir) m.dirFlips++;
      prevNetDir = nd;
    }
    if (kind === "reverse" && !inReverse) {
      m.reverseRuns++;
      inReverse = true;
    } else if (kind === "forward") {
      inReverse = false;
    }

    if (!frontFresh) m.staleFrames++;

    // UNSAFE: driving net-forward while a fresh obstacle is near dead-ahead.
    if (frontFresh && front < NEAR_CM && L + R > SPIN_NET) m.forwardIntoNear++;

    // LIVENESS: everything open but we are not translating forward.
    const allOpen =
      frontFresh &&
      front > OPEN_CM &&
      (leftC.count === 0 || leftC.median > NEAR_CM) &&
      (rightC.count === 0 || rightC.median > NEAR_CM);
    if (allOpen && L + R <= SPIN_NET) m.frozenInOpen++;

    // Chatter.
    if (s.avoidDir !== 0 && prevAvoid !== 0 && Math.sign(s.avoidDir) !== Math.sign(prevAvoid))
      m.avoidFlips++;
    prevAvoid = s.avoidDir;
    const steerSign = Math.sign(R - L);
    if (kind === "forward" && steerSign !== 0 && prevSteerSign !== 0 && steerSign !== prevSteerSign)
      m.steerSignFlips++;
    if (kind === "forward") prevSteerSign = steerSign;

    m.frames++;
  }

  m.bowFinal = s.nav.bowLocked ? Math.round(s.nav.bowOffsetDeg * 10) / 10 : null;
  return m;
}

function pct(n, d) {
  return d ? ((100 * n) / d).toFixed(1) + "%" : "-";
}

const files = fs
  .readdirSync(LOG_DIR)
  .filter((f) => f.startsWith("nav-") && f.endsWith(".ndjson"))
  .sort();

const rows = [];
for (const f of files) {
  const m = replayFile(f);
  if (m && m.frames > 0) rows.push(m);
}

// ---- Report ----
console.log("\n=== NAV REPLAY on real-hardware telemetry ===\n");
const agg = {
  frames: 0,
  forwardIntoNear: 0,
  frozenInOpen: 0,
  spinFrames: 0,
  reverseFrames: 0,
  forwardFrames: 0,
  holdFrames: 0,
  staleFrames: 0,
  avoidFlips: 0,
  steerSignFlips: 0,
  locked: 0,
  bowJumps: 0,
  wrongWayPivot: 0,
  dirFlips: 0,
  reverseRuns: 0,
};
for (const m of rows) {
  console.log(`${m.file}  (${m.frames} frames)`);
  console.log(
    `   bow: ${m.bowFinal == null ? "NEVER LOCKED" : m.bowFinal + "\u00b0 @frame " + m.bowLockedFrame}` +
      `  jumps=${m.bowJumps}`
  );
  console.log(
    `   drive: fwd ${pct(m.forwardFrames, m.frames)}  spin ${pct(m.spinFrames, m.frames)}` +
      `  rev ${pct(m.reverseFrames, m.frames)}  hold ${pct(m.holdFrames, m.frames)}  stale ${pct(m.staleFrames, m.frames)}`
  );
  console.log(
    `   UNSAFE fwd-into-near: ${m.forwardIntoNear}   LIVENESS frozen-in-open: ${m.frozenInOpen}` +
      `   wrong-way-pivot: ${m.wrongWayPivot}` +
      `   back-and-forth: dirFlips ${m.dirFlips} reverseRuns ${m.reverseRuns}` +
      `   chatter: avoidFlips ${m.avoidFlips} steerFlips ${m.steerSignFlips}`
  );
  for (const k of Object.keys(agg)) if (k in m) agg[k] += m[k];
  agg.locked += m.bowFinal == null ? 0 : 1;
  agg.frames += 0; // already added via loop (frames counted below)
}
// fix frames double
agg.frames = rows.reduce((a, m) => a + m.frames, 0);

console.log("\n--- AGGREGATE ---");
console.log(`files ${rows.length}, frames ${agg.frames}, bow-locked in ${agg.locked}/${rows.length} runs`);
console.log(
  `drive: fwd ${pct(agg.forwardFrames, agg.frames)}  spin ${pct(agg.spinFrames, agg.frames)}` +
    `  rev ${pct(agg.reverseFrames, agg.frames)}  hold ${pct(agg.holdFrames, agg.frames)}  stale ${pct(agg.staleFrames, agg.frames)}`
);
console.log(`UNSAFE fwd-into-near: ${agg.forwardIntoNear}  (${pct(agg.forwardIntoNear, agg.frames)})`);
console.log(`LIVENESS frozen-in-open: ${agg.frozenInOpen}  (${pct(agg.frozenInOpen, agg.frames)})`);
console.log(`WRONG-WAY pivots (into wall): ${agg.wrongWayPivot}`);
console.log(`BACK-AND-FORTH: dirFlips ${agg.dirFlips}  reverseRuns ${agg.reverseRuns}`);
console.log(`chatter: avoidFlips ${agg.avoidFlips}  steerFlips ${agg.steerSignFlips}  bowJumps ${agg.bowJumps}`);
console.log("");
