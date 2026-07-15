// Diagnostic: recover the TRUE fixed bow offset from real logs.
// The bow (servo-frame) is fixed hardware, so it must be consistent across all
// runs. For each frame where the boat was actually driving roughly straight
// forward (using the LOGGED command that produced the telemetry), we measure the
// range-closing rate per raw bearing bin. The raw bearing that consistently
// closes fastest IS the bow. Aggregates across every log to expose the true peak.

const fs = require("fs");
const path = require("path");
const LOG_DIR = path.join(__dirname, "logs");

const BEAMS = [
  { dir: 0, key: "usFront" },
  { dir: 90, key: "usRight" },
  { dir: 180, key: "usRadar" },
  { dir: 270, key: "usLeft" },
];
const BIN = 15;
const MAXR = 300;
function norm(d) {
  let o = d % 360;
  if (o < 0) o += 360;
  return o;
}
function binOf(deg) {
  return norm(Math.round(norm(deg) / BIN) * BIN) % 360;
}

// accumulate closing-rate samples per raw bin, across all logs
const votes = new Map(); // bin -> { sum, n, closeSum }

const files = fs
  .readdirSync(LOG_DIR)
  .filter((f) => f.startsWith("nav-") && f.endsWith(".ndjson"))
  .sort();

let straightFrames = 0;
for (const f of files) {
  const lines = fs.readFileSync(path.join(LOG_DIR, f), "utf8").trim().split("\n").filter(Boolean);
  const prev = new Map(); // bin -> {dist,t}
  for (const line of lines) {
    let r;
    try {
      r = JSON.parse(line);
    } catch {
      continue;
    }
    if (!r.tel || !r.cmd) continue;
    const L = r.cmd.left,
      R = r.cmd.right;
    const straight = L > 0 && R > 0 && L + R >= 120 && Math.abs(L - R) <= 40;
    const servo = r.tel.radarAngle ?? 0;
    for (const b of BEAMS) {
      let raw = r.tel[b.key];
      if (raw == null || raw < 0) continue;
      const meas = raw === 0 || raw >= 999 ? MAXR : raw;
      const bin = binOf(b.dir + servo);
      const p = prev.get(bin);
      prev.set(bin, { dist: meas, t: r.t });
      if (straight && p) {
        const dt = (r.t - p.t) / 1000;
        if (dt >= 0.1 && dt <= 4) {
          const rate = (meas - p.dist) / dt; // neg = closing
          if (rate < -5 && rate > -130) {
            const v = votes.get(bin) || { sum: 0, n: 0, closeSum: 0 };
            v.sum += rate;
            v.n++;
            v.closeSum += -rate;
            votes.set(bin, v);
          }
        }
      }
    }
    if (straight) straightFrames++;
  }
}

console.log(`\nstraight-forward frames: ${straightFrames}`);
console.log("\nraw-bin | votes | avg closing rate (cm/s) | total closing weight");
const arr = [...votes.entries()].sort((a, b) => a[0] - b[0]);
let bestBin = null,
  bestWeight = -1;
// circular accumulator (same as the online estimator) over ALL data
let accSin = 0,
  accCos = 0;
for (const [bin, v] of arr) {
  const avg = v.sum / v.n;
  console.log(
    `  ${String(bin).padStart(3)}   |  ${String(v.n).padStart(4)} |  ${avg.toFixed(1).padStart(7)}` +
      `             | ${v.closeSum.toFixed(0)}`
  );
  if (v.closeSum > bestWeight) {
    bestWeight = v.closeSum;
    bestBin = bin;
  }
  const rad = (bin * Math.PI) / 180;
  accSin += v.closeSum * Math.sin(rad);
  accCos += v.closeSum * Math.cos(rad);
}
const vectorBow = norm((Math.atan2(accSin, accCos) * 180) / Math.PI);
console.log(`\nSINGLE fastest-closing raw bin (=bow candidate): ${bestBin}\u00b0`);
console.log(`VECTOR-SUM bow estimate over all data: ${vectorBow.toFixed(1)}\u00b0`);
console.log("");
