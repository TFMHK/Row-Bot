#!/usr/bin/env python3
"""Continuous motion-calibration harness for the real boat.

Design goal: MAXIMISE useful motion per minute on the slow (~0.2-1 Hz),
motion-gated RF link. Instead of the old "map -> pulse -> stop -> analyse ->
repeat" loop (mostly idle), this runs the WHOLE maneuver timeline back-to-back
in one pass, logging every FRESH telemetry frame (deduped on lastMessageAt)
with a phase tag. Nothing is analysed during the run.

Two modes:
  run      -- connect, drive the timeline, stream a JSONL log. No analysis.
  analyze  -- read a JSONL log and print the calibration results at the end.

Usage:
  python motion_experiment.py run   [--port COM4] [--base http://127.0.0.1:8765]
  python motion_experiment.py analyze <logfile.jsonl>

Log record (one per fresh frame):
  {"t":rel_s,"phase":str,"l":cmdL,"r":cmdR,"ra":cmdAngle,
   "F":usFront,"R":usRight,"L":usLeft,"B":usRadar,"tra":teleAngle,"stale":bool}
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.request
from pathlib import Path

# --- Timeline: (label, leftSpeed, rightSpeed, duration_s) --------------------
# Rest windows (0,0) let the idle-sweep build a full radar map for pose-diff.
# Motion pulses are short and gentle; each is followed by a coast+rest window
# so the link recovers and the after-map is dense. CCW was missing before.
TIMELINE = [
    ("rest0",      0,   0, 6.0),   # initial map
    ("fwd",       80,  80, 2.0),   # cruise (speed + bow offset)
    ("rest_fwd",   0,   0, 5.0),   # forward coast + map
    ("cw",        80, -80, 2.0),   # clockwise pivot
    ("rest_cw",    0,   0, 5.0),   # cw coast + map
    ("ccw",      -80,  80, 2.0),   # counter-clockwise pivot
    ("rest_ccw",   0,   0, 5.0),   # ccw coast + map
    ("cw_short",  80, -80, 1.0),   # short cw (rate vs duration)
    ("rest_cws",   0,   0, 5.0),
    ("arc",       80,  40, 2.0),   # gentle forward arc (steer-while-moving)
    ("rest_arc",   0,   0, 5.0),
    ("end",        0,   0, 1.0),
]

SWEEP_ANGLES = [0, 30, 60, 90, 120, 150, 180]
CMD_PERIOD_S = 0.2        # how often we POST a command (server also rate-limits)
POLL_PERIOD_S = 0.05      # how often we poll /api/state
ABORT_FRONT_CM = 22       # during forward/arc, stop if a wall gets this close
BIN_COUNT = 24            # 15 deg per bin
BIN_DEG = 360 // BIN_COUNT


# --------------------------------------------------------------------------- #
# HTTP helpers
# --------------------------------------------------------------------------- #
def _post(base: str, path: str, payload: dict) -> dict:
    req = urllib.request.Request(
        base + path,
        data=json.dumps(payload).encode("ascii"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode())


def _get(base: str, path: str) -> dict:
    with urllib.request.urlopen(base + path, timeout=5) as resp:
        return json.loads(resp.read().decode())


def _send_cmd(base: str, l: int, r: int, ra: int) -> None:
    try:
        _post(base, "/api/command",
              {"leftSpeed": l, "rightSpeed": r, "winchSpeed": 0, "radarAngle": ra})
    except Exception:
        pass


# --------------------------------------------------------------------------- #
# RUN: drive the timeline, log fresh frames, no analysis
# --------------------------------------------------------------------------- #
def run(base: str, port: str) -> None:
    # Connect (best effort; if already connected the server just re-opens).
    try:
        res = _post(base, "/api/connect", {"port": port})
        if "error" in res:
            print(f"CONNECT ERROR: {res['error']}", file=sys.stderr)
            return
    except Exception as exc:
        print(f"CONNECT FAILED: {exc}", file=sys.stderr)
        return

    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = Path(__file__).resolve().parent / f"motion_log_{stamp}.jsonl"
    total = sum(p[3] for p in TIMELINE)
    print(f"Running timeline (~{total:.0f}s). Log: {log_path}")

    t0 = time.time()
    last_cmd = 0.0
    last_seen_ts = None
    sweep_i = 0
    aborted_phase = None
    fresh = 0

    # Precompute phase boundaries.
    bounds = []
    acc = 0.0
    for label, l, r, dur in TIMELINE:
        bounds.append((acc, acc + dur, label, l, r))
        acc += dur

    def phase_at(elapsed: float):
        for start, end, label, l, r in bounds:
            if start <= elapsed < end:
                return label, l, r
        return "end", 0, 0

    with open(log_path, "w", encoding="utf-8") as fh:
        try:
            while True:
                now = time.time()
                elapsed = now - t0
                if elapsed >= total:
                    break
                label, l, r = phase_at(elapsed)

                # Safety abort: if a wall is close during forward-ish motion,
                # cut drive for the remainder of that phase.
                if label == aborted_phase:
                    l, r = 0, 0

                # Send a command every CMD_PERIOD_S, cycling radarAngle so the
                # servo keeps sweeping and we build bearings across the run.
                if now - last_cmd >= CMD_PERIOD_S:
                    ra = SWEEP_ANGLES[sweep_i % len(SWEEP_ANGLES)]
                    sweep_i += 1
                    _send_cmd(base, l, r, ra)
                    last_cmd = now
                    cur_ra = ra
                else:
                    cur_ra = SWEEP_ANGLES[(sweep_i - 1) % len(SWEEP_ANGLES)]

                # Poll state; log only genuinely fresh frames.
                try:
                    st = _get(base, "/api/state")["state"]
                except Exception:
                    time.sleep(POLL_PERIOD_S)
                    continue
                ts = st.get("lastMessageAt")
                tel = st.get("telemetry", {})
                if ts is not None and ts != last_seen_ts:
                    last_seen_ts = ts
                    fresh += 1
                    rec = {
                        "t": round(elapsed, 3), "phase": label,
                        "l": l, "r": r, "ra": cur_ra,
                        "F": tel.get("usFront"), "R": tel.get("usRight"),
                        "L": tel.get("usLeft"), "B": tel.get("usRadar"),
                        "tra": tel.get("radarAngle"), "stale": st.get("stale"),
                    }
                    fh.write(json.dumps(rec) + "\n")
                    # Front-wall abort check on live fresh data.
                    if label in ("fwd", "arc") and aborted_phase != label:
                        f = tel.get("usFront")
                        if isinstance(f, int) and 0 < f < ABORT_FRONT_CM:
                            aborted_phase = label
                            _send_cmd(base, 0, 0, cur_ra)

                time.sleep(POLL_PERIOD_S)
        finally:
            # Always stop the boat.
            for _ in range(3):
                _send_cmd(base, 0, 0, 90)
                time.sleep(0.1)

    print(f"DONE. fresh_frames={fresh}. Analyse with:\n"
          f"  python {Path(__file__).name} analyze {log_path.name}")


# --------------------------------------------------------------------------- #
# ANALYZE: read the log, compute results (offline, at the end)
# --------------------------------------------------------------------------- #
def _rel_bearings(rec: dict):
    """Yield (bearing_deg, distance_cm) for the 4 sensors of one frame.

    Sensors are 90 deg apart on the servo; the servo angle (tra) rotates all
    four. The absolute servo-zero->bow offset is unknown but CANCELS in
    pose-differencing (we compare shifts), so we leave it at 0 here.
      B(usRadar)=back=+180, F(usFront)=+0, L(usLeft)=+270, R(usRight)=+90
    """
    ra = rec.get("tra")
    if ra is None:
        return
    for key, off in (("F", 0), ("R", 90), ("B", 180), ("L", 270)):
        d = rec.get(key)
        if isinstance(d, int) and 0 < d < 999:
            yield (ra + off) % 360, d


def _build_map(records) -> dict:
    """Fold frames into bin -> min distance (walls = nearest point)."""
    m: dict[int, int] = {}
    for rec in records:
        for bearing, dist in _rel_bearings(rec):
            b = int(round(bearing / BIN_DEG)) % BIN_COUNT
            if b not in m or dist < m[b]:
                m[b] = dist
    return m


def _best_shift(before: dict, after: dict):
    """Find rotation (deg, multiple of BIN_DEG) minimising wall-map residual."""
    best = (None, None, 0)  # (shift_deg, avg_residual, overlap)
    for steps in range(BIN_COUNT):
        res, overlap = 0.0, 0
        for b, d in after.items():
            src = (b - steps) % BIN_COUNT
            if src in before:
                res += abs(d - before[src])
                overlap += 1
        if overlap >= 3:
            avg = res / overlap
            if best[1] is None or avg < best[1]:
                best = ((steps * BIN_DEG + 180) % 360 - 180, round(avg, 1), overlap)
    return best


def _rest_before(recs, motion_label):
    """Records of the rest window immediately BEFORE a motion phase."""
    order = [p[0] for p in TIMELINE]
    idx = order.index(motion_label)
    prev = order[idx - 1] if idx > 0 else None
    return [r for r in recs if r["phase"] == prev] if prev else []


def _rest_after(recs, motion_label):
    order = [p[0] for p in TIMELINE]
    idx = order.index(motion_label)
    nxt = order[idx + 1] if idx + 1 < len(order) else None
    return [r for r in recs if r["phase"] == nxt] if nxt else []


def analyze(log_file: str) -> None:
    path = Path(log_file)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / log_file
    recs = [json.loads(ln) for ln in path.read_text(encoding="utf-8").splitlines() if ln.strip()]
    print(f"# Analysis of {path.name} ({len(recs)} fresh frames)\n")

    # Frame density per phase (diagnoses starvation).
    from collections import Counter
    dens = Counter(r["phase"] for r in recs)
    print("Frames per phase:", dict(dens), "\n")

    # Rotation phases: pose-diff shift (rest-before vs rest-after).
    for label in ("cw", "ccw", "cw_short", "arc"):
        before = _build_map(_rest_before(recs, label))
        after = _build_map(_rest_after(recs, label))
        shift, avg, overlap = _best_shift(before, after)
        dur = next(p[3] for p in TIMELINE if p[0] == label)
        rate = (shift / dur) if (shift is not None and dur) else None
        print(f"[{label}] dur={dur}s shift={shift} deg "
              f"rate~={rate if rate is None else round(rate,1)} deg/s "
              f"overlap={overlap} residual={avg}")
    print()

    # Forward: bow bearing + cruise speed from the closing wall.
    fwd = [r for r in recs if r["phase"] == "fwd"]
    if fwd:
        # Bin that closes fastest between first and last third = bow direction.
        first = _build_map(fwd[: max(1, len(fwd) // 3)])
        last = _build_map(fwd[-max(1, len(fwd) // 3):])
        deltas = {b: last[b] - first[b] for b in last if b in first}
        if deltas:
            bow_bin = min(deltas, key=lambda b: deltas[b])
            print(f"[fwd] bow bin ~= {bow_bin} (~{bow_bin*BIN_DEG} deg in servo frame), "
                  f"closed {deltas[bow_bin]} cm")
            # Cruise speed: front-bin distance vs t slope.
            series = [(r["t"], r["F"]) for r in fwd if isinstance(r.get("F"), int) and 0 < r["F"] < 999]
            if len(series) >= 2:
                dt = series[-1][0] - series[0][0]
                dd = series[0][1] - series[-1][1]
                spd = (dd / dt) if dt > 0 else None
                print(f"[fwd] front closed {dd} cm over {dt:.2f}s -> "
                      f"~{spd if spd is None else round(spd,1)} cm/s (rough)")
    print("\n(Reminder: 15deg FOV -> bearings +/-7.5deg; obstacles are WALLS; "
          "shifts include yaw coast; residual high => hull translates while turning.)")


def main() -> None:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    pr = sub.add_parser("run")
    pr.add_argument("--port", default="COM4")
    pr.add_argument("--base", default="http://127.0.0.1:8765")
    pa = sub.add_parser("analyze")
    pa.add_argument("logfile")
    args = ap.parse_args()
    if args.cmd == "run":
        run(args.base, args.port)
    else:
        analyze(args.logfile)


if __name__ == "__main__":
    main()
