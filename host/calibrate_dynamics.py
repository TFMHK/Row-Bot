#!/usr/bin/env python3
"""Short, focused real-world dynamics calibration for the boat (pose-diff).

The real telemetry link is slow (~0.2 Hz of *distinct* sensor frames) and during
motion it is dominated by keep-alive ack packets, so we CANNOT track yaw
continuously. Instead we use pose-differencing, which only needs good data at
rest (where the server's idle radar sweep keeps pings flowing):

  1. build a full polar wall-map at rest (BEFORE),
  2. apply ONE clean motor pulse with minimal RF traffic (send once; the server
     heartbeat keeps it alive so we don't starve the link),
  3. let the hull settle (captures coast),
  4. build a polar wall-map at rest (AFTER),
  5. the angular shift that best re-aligns the two maps == net boat rotation for
     that pulse (steady turn + spin-up lag + coast, lumped).

Estimated quantities:
  * bow direction vs servo-zero offset, cruise speed, motor bias  [forward test]
  * net yaw per pulse, CW vs CCW asymmetry                         [spin tests]
  * steady yaw rate + spin-up/coast, by differencing two pulse lengths.

Radar interpretation: every obstacle is a WALL, so each polar bin keeps the
NEAREST return (walls occlude). Each beam is a 15 deg cone, so bins are 15 deg
wide. The four sensors sit 90 deg apart on the sweep servo: F@ra, R@ra+90,
B@ra+180, L@ra+270 (servo-zero offset is unknown but constant, so it cancels in
the BEFORE/AFTER difference).

Usage:  python host/calibrate_dynamics.py [--port COM4]
"""
from __future__ import annotations

import argparse
import json
import threading
import time
import urllib.request

BASE = "http://127.0.0.1:8765"

FWD = 80             # forward drive magnitude for the cruise test
TURN = 80            # differential magnitude for the spin tests
FRONT_STOP_CM = 26   # abort forward motion if the bow wall gets this close
MIN_FRAMES = 12      # keep mapping until this many DISTINCT frames are gathered
MAX_MAP_SECONDS = 26.0   # ...but never map longer than this (link may be slow)
SETTLE_SECONDS = 6.0 # post-pulse coast/settle + link recovery before mapping
BIN_DEG = 15         # polar bin width == beam FOV
NBINS = 360 // BIN_DEG
OUT_OF_RANGE = 999


def _post(path: str, data: dict) -> dict:
    req = urllib.request.Request(
        BASE + path, data=json.dumps(data).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=2) as r:
            return json.load(r)
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


def _get(path: str) -> dict:
    try:
        with urllib.request.urlopen(BASE + path, timeout=2) as r:
            return json.load(r)
    except Exception as exc:  # noqa: BLE001
        return {"error": str(exc)}


def _bin(angle: float) -> int:
    return int(round((angle % 360) / BIN_DEG)) % NBINS


class Rig:
    def __init__(self) -> None:
        self.stop = threading.Event()
        self.t0 = time.time()
        self.latest_front = OUT_OF_RANGE
        self._last_ts = None
        self._frames: list[dict] = []     # distinct fresh frames
        self._lock = threading.Lock()
        self._sampler = threading.Thread(target=self._sample_loop, daemon=True)

    # --- telemetry sampling ------------------------------------------------
    def _sample_loop(self) -> None:
        while not self.stop.is_set():
            s = _get("/api/state").get("state")
            if s:
                ts = s.get("lastMessageAt")
                t = s.get("telemetry", {})
                # Distinct-frame test: ignore ack/keep-alive repeats that bump
                # lastMessageAt but carry no new sensor reading.
                key = (t.get("usFront"), t.get("usRight"), t.get("usLeft"),
                       t.get("usRadar"), t.get("radarAngle"))
                if ts is not None and ts != self._last_ts:
                    self._last_ts = ts
                    self.latest_front = t.get("usFront") or OUT_OF_RANGE
                    with self._lock:
                        if not self._frames or self._frames[-1]["key"] != key:
                            self._frames.append({
                                "t": round(time.time() - self.t0, 2),
                                "key": key, "ra": t.get("radarAngle"),
                                "F": t.get("usFront"), "R": t.get("usRight"),
                                "L": t.get("usLeft"), "B": t.get("usRadar"),
                            })
            time.sleep(0.05)

    def start(self) -> None:
        self._sampler.start()

    def finish(self) -> None:
        self.stop.set()
        self._sampler.join(timeout=1.0)

    def cmd(self, left: int, right: int, radar: int = 90) -> None:
        _post("/api/command", {"leftSpeed": left, "rightSpeed": right,
                               "winchSpeed": 0, "radarAngle": radar})

    # --- building blocks ---------------------------------------------------
    def rest_map(self, label: str) -> dict[int, float]:
        """Adaptively gather frames at rest and fold them into a polar map.

        Sends NO drive commands (server idle-sweep supplies the scan), so the
        link is not starved. Keeps sampling until MIN_FRAMES distinct frames are
        collected or MAX_MAP_SECONDS elapses -- this guarantees a dense map even
        right after a pulse, when the link is briefly starved. Each bin keeps the
        nearest wall return seen in that bin.
        """
        self.cmd(0, 0, 90)          # ensure stopped -> server does idle sweep
        with self._lock:
            start_i = len(self._frames)
        t_end = time.time() + MAX_MAP_SECONDS
        while time.time() < t_end:
            with self._lock:
                n = len(self._frames) - start_i
            if n >= MIN_FRAMES:
                break
            time.sleep(0.3)
        with self._lock:
            frames = self._frames[start_i:]
        polar: dict[int, float] = {}
        for f in frames:
            ra = f["ra"]
            if ra is None:
                continue
            for sensor, off in (("F", 0), ("R", 90), ("B", 180), ("L", 270)):
                d = f[sensor]
                if d is None or d >= OUT_OF_RANGE:
                    continue
                b = _bin(ra + off)
                if b not in polar or d < polar[b]:
                    polar[b] = d
        print(json.dumps({"map": label, "frames": len(frames),
                          "bins": {str(k): round(v) for k, v in sorted(polar.items())}}))
        return polar

    def pulse(self, label: str, left: int, right: int, dur: float,
              guard_forward: bool = False) -> None:
        """One clean pulse: send once, keep alive via heartbeat, then stop."""
        print(json.dumps({"pulse": label, "l": left, "r": right, "dur": dur}))
        self.cmd(left, right, 90)
        end = time.time() + dur
        while time.time() < end:
            if guard_forward and self.latest_front is not None and \
                    self.latest_front < FRONT_STOP_CM:
                print(json.dumps({"pulse": label, "ABORT_wall": self.latest_front}))
                break
            time.sleep(0.1)
        self.cmd(0, 0, 90)


def best_shift(before: dict[int, float], after: dict[int, float]) -> dict:
    """Angular shift (deg) that best re-aligns AFTER onto BEFORE == boat yaw.

    Positive shift == walls moved to higher bearing == boat rotated negative
    (opposite). We report the raw shift and let the caller interpret sign vs the
    commanded direction.
    """
    best = None
    for step in range(-NBINS // 2, NBINS // 2 + 1):
        num = 0
        cost = 0.0
        for b, dv in before.items():
            nb = (b + step) % NBINS
            if nb in after:
                cost += abs(dv - after[nb])
                num += 1
        if num >= 3:
            avg = cost / num
            if best is None or avg < best["avg"]:
                best = {"shift_deg": step * BIN_DEG, "avg": round(avg, 1),
                        "overlap": num}
    return best or {"shift_deg": None, "avg": None, "overlap": 0}


def run(port: str) -> None:
    st = _get("/api/state").get("state", {})
    if not st.get("connected"):
        _post("/api/connect", {"port": port})
        time.sleep(2.5)

    rig = Rig()
    rig.start()

    # ---- Forward test: bow direction, cruise speed, drift ----
    m0 = rig.rest_map("FWD_before")
    rig.pulse("forward", FWD, FWD, 2.5, guard_forward=True)
    time.sleep(SETTLE_SECONDS)
    m1 = rig.rest_map("FWD_after")
    fwd_delta = {str(b): round(m1[b] - m0[b])
                 for b in sorted(set(m0) & set(m1))}
    print(json.dumps({"result": "forward_bin_delta_cm(after-before)", "d": fwd_delta}))

    # ---- CW long spin ----
    c0 = rig.rest_map("CW_before")
    rig.pulse("cw_2.5s", TURN, -TURN, 2.5)
    time.sleep(SETTLE_SECONDS)
    c1 = rig.rest_map("CW_after")
    print(json.dumps({"result": "cw_2.5s_shift", **best_shift(c0, c1)}))

    # ---- CW short spin (with CW long -> spin-up/coast separation) ----
    cs0 = rig.rest_map("CWs_before")
    rig.pulse("cw_1.0s", TURN, -TURN, 1.0)
    time.sleep(SETTLE_SECONDS)
    cs1 = rig.rest_map("CWs_after")
    print(json.dumps({"result": "cw_1.0s_shift", **best_shift(cs0, cs1)}))

    # ---- CCW long spin ----
    w0 = rig.rest_map("CCW_before")
    rig.pulse("ccw_2.5s", -TURN, TURN, 2.5)
    time.sleep(SETTLE_SECONDS)
    w1 = rig.rest_map("CCW_after")
    print(json.dumps({"result": "ccw_2.5s_shift", **best_shift(w0, w1)}))

    rig.cmd(0, 0, 90)
    rig.finish()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    args = ap.parse_args()
    run(args.port)
