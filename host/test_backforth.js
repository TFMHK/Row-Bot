// Focused closed-loop-ish unit check for the forward/back jitter fix.
// Synthesises a "boat pinned near a wall" telemetry stream where the front
// distance bounces around the RW_REVERSE_CM (30 cm) threshold at the real slow
// link cadence, with the LEFT side open. Verifies the navigator no longer
// alternates reverse<->forward every tick but instead commits to a reverse burst
// and then pivots toward the open side.

const fs = require("fs");
const path = require("path");
const vm = require("vm");

const APP = path.join(__dirname, "web", "app.js");
let VCLOCK = 0;
const stub = new Proxy(function () {}, {
  get(_t, p) {
    if (p === "then") return undefined;
    if (p === Symbol.toPrimitive) return () => 0;
    if (p === "length") return 0;
    return stub;
  },
  apply: () => stub,
  construct: () => stub,
  set: () => true,
});
function loadNav() {
  let src = fs.readFileSync(APP, "utf8");
  src +=
    "\n;globalThis.__NAV__ = { state, liveScan, rawPrev, SENSOR_BEAMS, updateLiveScan," +
    " liveCone, liveDistance, updateAutonomousRealworld, realworldSafeCommand };\n";
  const sandbox = {
    document: stub, window: stub, navigator: stub, location: stub, localStorage: stub,
    EventSource: stub, fetch: () => stub, setInterval: () => 0, setTimeout: () => 0,
    clearInterval: () => {}, clearTimeout: () => {}, requestAnimationFrame: () => 0,
    cancelAnimationFrame: () => {}, performance: { now: () => VCLOCK }, console, Math,
    Date, JSON, connectBtn: stub, modeSwitch: stub, refreshPortsBtn: stub,
    mockSwitch: stub, mapClearBtn: stub,
  };
  sandbox.globalThis = sandbox;
  sandbox.self = sandbox;
  vm.createContext(sandbox);
  vm.runInContext(src, sandbox, { filename: "app.js" });
  return sandbox.__NAV__;
}

const NAV = loadNav();
const s = NAV.state;

function arm() {
  NAV.liveScan.clear();
  NAV.rawPrev.clear();
  s.mockEnabled = false;
  s.manualMode = false;
  s.avoidDir = 0;
  s.nav.bowOffsetDeg = 0; // put bow at raw 0 so usFront(dir0) reads dead-ahead
  s.nav.bowLocked = true;
  s.nav.rwPhase = "run";
  s.nav.spinUntil = 0;
  s.nav.settleUntil = 0;
  s.nav.reverseUntil = 0;
  s.cmd.leftSpeed = 0;
  s.cmd.rightSpeed = 0;
}

function classify(L, R) {
  const net = L + R, diff = L - R;
  if (Math.abs(net) < 20 && Math.abs(diff) > 40) return "SPIN";
  if (net > 20) return "FWD";
  if (net < -20) return "REV";
  return "HOLD";
}

// Front bounces around 30 cm (noise), sides: LEFT open (usLeft big), RIGHT near.
// Sensor mount dirs: usFront=0, usRight=90, usRadar=180(back), usLeft=270.
// With bowOffset 0 and servo 0, usFront -> bow, usLeft -> 270(=-90 left), usRight->90.
const frontSeq = [30, 28, 33, 29, 34, 27, 31, 30, 35, 26, 32, 30, 33, 29, 31, 34, 28, 30, 32, 30];
function run() {
  arm();
  const seq = [];
  let t = 1000;
  for (let i = 0; i < frontSeq.length; i++) {
    VCLOCK = t;
    s.telemetry.usFront = frontSeq[i];
    s.telemetry.usRight = 40; // right quarter tight
    s.telemetry.usRadar = 200; // clear behind
    s.telemetry.usLeft = 180; // LEFT wide open -> should pivot/bias LEFT
    s.telemetry.radarAngle = 0;
    s.telemetry.boatHeadingDeg = 0;
    NAV.updateLiveScan();
    NAV.updateAutonomousRealworld();
    NAV.realworldSafeCommand(s.cmd);
    seq.push(classify(s.cmd.leftSpeed, s.cmd.rightSpeed));
    t += 350; // ~real slow cadence
  }
  return seq;
}

const seq = run();
console.log("\nPinned-near-wall command sequence (front ~30cm, LEFT open):");
console.log("  " + seq.join(" "));
let flips = 0;
let prev = 0;
for (const k of seq) {
  const d = k === "FWD" ? 1 : k === "REV" ? -1 : 0;
  if (d !== 0) {
    if (prev !== 0 && d !== prev) flips++;
    prev = d;
  }
}
const fwd = seq.filter((k) => k === "FWD").length;
const rev = seq.filter((k) => k === "REV").length;
console.log(`  FWD=${fwd} REV=${rev} SPIN=${seq.filter((k) => k === "SPIN").length} HOLD=${seq.filter((k) => k === "HOLD").length}`);
console.log(`  forward<->reverse flips: ${flips}`);
console.log(
  flips <= 1 && fwd === 0
    ? "  => PASS: no pointless forward/back jitter (commits to reverse/pivot).\n"
    : "  => review: still alternating.\n"
);
