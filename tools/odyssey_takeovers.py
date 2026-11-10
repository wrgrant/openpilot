#!/usr/bin/env python3
"""Why the driver held the wheel: every real wheel press while openpilot steered, sorted by cause, and how far the wheel
lagged the request at low speed. This is the outcome measure: hands-off driving means these counts go to zero.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_takeovers.py ~/comma-logs/<dongle>_<route> [more routes] [-v]

Presses (above 18 mph, steering on, no blinker, at least 0.3 s; shorter ones are torque-sensor blips, not a hand):
  curve, at the limit   the driver added turn and the command had been at 100% for most of the second before:
                        the EPS was out of torque
  curve, adding turn    the driver added turn with torque to spare (often a second grab in the same curve)
  curve, easing turn    the driver steered against the turn
  straight              the model asked for less than 0.5 m/s2
Low speed (below 25 mph, steering on): share of time the wheel was more than 5 degrees short of the angle the request
needs, and, when it was short with hands off, the command it was getting. Short at 100% is the EPS; short at a partial
command is the controller. A drive that ran SteerLowSpeedAngleAB is split by arm.

Signs (verified on drives 1a7-1ae): steeringTorque and steeringAngleDeg share a sign (r = 0.8-0.9 in hand driving), and
openpilot's curvature has the opposite sign to both (r = -0.98). Each route's 100 Hz table is cached in the route folder
as takeovers.npz.
"""
import argparse
import glob
import json
import os

import numpy as np

FS = 100
MIN_PRESS_S = 0.3
V_PRESS = 8.0  # m/s (18 mph): below this nearly every press is the driver making an intersection turn
CURVE_ACCEL = 0.5  # m/s2 requested: a curve
SHORT_DEG = 5.0
KINDS = ("curve, at the limit", "curve, adding turn", "curve, easing turn", "straight")
BANDS = ((0.5, 4.5, "<10"), (4.5, 8.0, "10-18"), (8.0, 11.2, "18-25"))
COLS = ("t", "seg", "v", "lat", "pressed", "drv", "blinker", "angle", "torque", "des_curv", "cur_curv")


def load(route_dir: str) -> dict:
  from openpilot.tools.lib.logreader import LogReader
  segs = sorted(glob.glob(os.path.join(route_dir, "*--*")), key=lambda d: int(d.rsplit("--", 1)[1]))
  cache = os.path.join(route_dir, "takeovers.npz")
  if os.path.exists(cache):
    d = dict(np.load(cache))
    if int(d["segments"]) == len(segs):
      return d
  rows, lowspeed_ab = [], None
  for path in segs:
    st = dict.fromkeys(COLS, 0.0)
    st["seg"] = int(path.rsplit("--", 1)[1])
    for m in sorted(LogReader(os.path.join(path, "rlog.zst")), key=lambda m: m.logMonoTime):
      w = m.which()
      if w == "initData" and lowspeed_ab is None:
        lowspeed_ab = any(e.key == "SteerLowSpeedAngleAB" and e.value == b"1" for e in m.initData.params.entries)
      elif w == "carState":
        cs = m.carState
        st.update(t=m.logMonoTime / 1e9, v=cs.vEgo, pressed=float(cs.steeringPressed), drv=cs.steeringTorque,
                  blinker=float(cs.leftBlinker or cs.rightBlinker), angle=cs.steeringAngleDeg)
        rows.append([st[c] for c in COLS])
      elif w == "carControl":
        st.update(lat=float(m.carControl.latActive), torque=m.carControl.actuators.torque, cur_curv=m.carControl.currentCurvature)
      elif w == "controlsState":
        st["des_curv"] = m.controlsState.desiredCurvature
  a = np.array(rows, dtype=float)
  d = {c: a[:, i] for i, c in enumerate(COLS)}
  d.update(segments=np.array(len(segs)), lowspeed_ab=np.array(bool(lowspeed_ab)))
  np.savez_compressed(cache, **d)
  return d


def runs(mask: np.ndarray, min_len: int) -> list[np.ndarray]:
  idx = np.flatnonzero(mask)
  return [r for r in np.split(idx, np.where(np.diff(idx) > 1)[0] + 1) if len(r) >= min_len] if len(idx) else []


def presses(d: dict) -> list[dict]:
  """Each real press with its cause. Curvature sign throughout (+ = right), so the driver's torque is negated."""
  v, lat = d["v"], d["lat"] > 0
  des = d["des_curv"] * v**2
  out = []
  for r in runs(d["pressed"] > 0, int(MIN_PRESS_S * FS)):
    a, b = r[0], r[-1] + 1
    if not lat[max(a - FS // 2, 0)] or v[a] < V_PRESS or d["blinker"][max(a - 3 * FS, 0):b + FS].max() > 0:
      continue
    before = slice(max(a - FS, 0), a)
    want = float(np.mean(des[before]))
    at_limit = float(np.mean(np.abs(d["torque"][before]) >= 0.99)) > 0.5
    driver = -float(np.mean(d["drv"][r]))
    if abs(want) < CURVE_ACCEL:
      kind = "straight"
    elif driver * want < 0:
      kind = "curve, easing turn"
    else:
      kind = "curve, at the limit" if at_limit else "curve, adding turn"
    out.append({"kind": kind, "seg": int(d["seg"][a]), "s": float(d["t"][a] - d["t"][np.flatnonzero(d["seg"] == d["seg"][a])[0]]),
                "mph": float(v[a] * 2.237), "dur": len(r) / FS, "want": want,
                "got": float(np.mean(d["cur_curv"][before] * v[before]**2))})
  return out


def low_speed(d: dict, arm: np.ndarray | None = None) -> list[tuple]:
  """Per speed band: (band, seconds steering, share of them more than SHORT_DEG short, hands-off-and-short seconds,
  median command then, share of those at 100%)."""
  v, lat = d["v"], d["lat"] > 0
  moving = v > 3
  deg_per_curv = np.linalg.lstsq(d["cur_curv"][moving, None], d["angle"][moving], rcond=None)[0][0]
  short = (d["des_curv"] - d["cur_curv"]) * deg_per_curv * np.sign(d["des_curv"] * deg_per_curv)
  hands_on = np.convolve(d["pressed"] > 0, np.ones(FS), mode="same") > 0
  out = []
  for lo, hi, band in BANDS:
    m = lat & (v >= lo) & (v < hi) & (arm if arm is not None else True)
    free = m & (short > SHORT_DEG) & ~hands_on
    cmd = np.abs(d["torque"][free])
    out.append((band, m.sum() / FS, float(np.mean(short[m] > SHORT_DEG)) if m.any() else np.nan, free.sum() / FS,
                float(np.median(cmd)) if len(cmd) else np.nan, float(np.mean(cmd >= 0.99)) if len(cmd) else np.nan))
  return out


def print_low_speed(rows: list[tuple], label: str) -> None:
  for band, secs, share, free_s, cmd, full in rows:
    if secs >= 5:
      print(f"  {label}{band:>6} mph: {secs:4.0f} s steering, wheel >{SHORT_DEG:.0f} deg short {share:4.0%} of it; "
            + f"hands off and short {free_s:3.0f} s, command median {cmd:.2f}, at 100% {full:.0%}")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir", nargs="+")
  ap.add_argument("-v", action="store_true", help="list every press")
  args = ap.parse_args()
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import LOWSPEED_ARMS, ab_arm

  total, minutes = dict.fromkeys(KINDS, 0), 0.0
  for path in args.route_dir:
    d = load(path)
    ps = presses(d)
    mins = float(((d["lat"] > 0) & (d["blinker"] == 0) & (d["v"] >= V_PRESS)).sum() / FS / 60)
    minutes += mins
    counts = {k: sum(p["kind"] == k for p in ps) for k in KINDS}
    for k in KINDS:
      total[k] += counts[k]
    print(f"\n== {os.path.basename(os.path.normpath(path))}: {len(ps)} presses in {mins:.1f} min of steering above 18 mph "
          + f"({10 * len(ps) / max(mins, 1e-9):.1f} per 10 min): " + ", ".join(f"{k} {n}" for k, n in counts.items()))
    if args.v:
      for p in ps:
        print(f"    seg {p['seg']:>2} +{p['s']:>4.1f} s  {p['kind']:<20} {p['mph']:>3.0f} mph, held {p['dur']:>4.1f} s, "
              + f"asked {p['want']:+.2f} m/s2, got {p['got']:+.2f}")
    if bool(d["lowspeed_ab"]):
      arm = np.array([ab_arm(t, LOWSPEED_ARMS) for t in d["t"]])
      for name in LOWSPEED_ARMS:
        print_low_speed(low_speed(d, arm == name), f"{name:>5} ")
    else:
      print_low_speed(low_speed(d), "")
  if len(args.route_dir) > 1:
    n = sum(total.values())
    print(f"\nall drives: {n} presses in {minutes:.1f} min ({10 * n / max(minutes, 1e-9):.1f} per 10 min): "
          + ", ".join(f"{k} {c}" for k, c in total.items()))
    print(json.dumps({"presses_per_10_min": round(10 * n / max(minutes, 1e-9), 2), "minutes": round(minutes, 1), **total}))


if __name__ == "__main__":
  main()
