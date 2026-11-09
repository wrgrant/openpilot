#!/usr/bin/env python3
"""Measure slow lane wander ("ping-pong", 7-20 s period) on a recorded drive, and say where it comes from.

For each stretch where openpilot steers on its own (engaged, no driver torque, no blinker, above --min-mph)
it reports, in the 7-20 s band:
  wander   lane-position swing (cm rms), from the model's lane lines, when both are visible
  period   strongest period in the band, from the desired lateral accel
  desired  how much the requested lateral accel swings (m/s^2 rms)
  error    how much actual lateral accel misses the request (m/s^2 rms)
  follow   error / desired: low (< ~0.3) means the controller follows the request and the swing is in what is
           asked for (model, planner, camera offset, steer delay); high means the controller or the steering
           itself (deadband, friction, saturation) is causing it
  lag      how far actual lateral accel trails desired, s

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_wander.py ~/comma-logs/<dongle>_<route> [--min-mph 25] [--min-s 45] [--ab]

--ab deadband|nnlc scores an A/B drive (SteerDeadbandCompAB or NnlcAB): it splits the stretches at the arm switches and
compares the arms.
"""
import argparse
import glob
import os

import numpy as np

FS = 20.0
BAND = (1 / 20, 1 / 7)  # Hz: 7-20 s periods


def load(route_dir: str) -> dict[str, np.ndarray]:
  from openpilot.tools.lib.logreader import LogReader
  cols = ("t", "seg", "v", "lat_active", "pressed", "blinker", "des_la", "act_la", "l_y", "r_y", "l_p", "r_p")
  rows = []
  segs = sorted(glob.glob(os.path.join(route_dir, "*--*")), key=lambda d: int(d.rsplit("--", 1)[1]))
  for d in segs:
    seg = int(d.rsplit("--", 1)[1])
    st = dict.fromkeys(cols, 0.0)
    st.update(seg=seg, l_y=np.nan, r_y=np.nan)
    last_t = -1.0
    for m in sorted(LogReader(os.path.join(d, "rlog.zst")), key=lambda m: m.logMonoTime):
      w = m.which()
      if w == "carState":
        cs = m.carState
        st.update(v=cs.vEgo, pressed=float(cs.steeringPressed), blinker=float(cs.leftBlinker or cs.rightBlinker))
        t = m.logMonoTime / 1e9
        if t - last_t >= 0.0495:  # 20 Hz rows, stamped on carState
          st["t"] = last_t = t
          rows.append([st[c] for c in cols])
      elif w == "carControl":
        st["lat_active"] = float(m.carControl.latActive)
      elif w == "controlsState":
        lcs = m.controlsState.lateralControlState
        if lcs.which() == "torqueState":
          st.update(des_la=lcs.torqueState.desiredLateralAccel, act_la=lcs.torqueState.actualLateralAccel)
      elif w == "modelV2" and len(m.modelV2.laneLines) == 4:
        st.update(l_y=m.modelV2.laneLines[1].y[0], r_y=m.modelV2.laneLines[2].y[0],
                  l_p=m.modelV2.laneLineProbs[1], r_p=m.modelV2.laneLineProbs[2])
  data = np.array(rows, dtype=float)
  return {c: data[:, i] for i, c in enumerate(cols)}


def bandpass(x: np.ndarray) -> np.ndarray:
  """Zero-phase band-pass by FFT mask, after removing the linear trend."""
  n = len(x)
  x = x - np.polyval(np.polyfit(np.arange(n), x, 1), np.arange(n))
  f = np.fft.rfftfreq(n, 1 / FS)
  X = np.fft.rfft(x)
  X[(f < BAND[0]) | (f > BAND[1])] = 0
  return np.fft.irfft(X, n)


def peak_period(x: np.ndarray) -> float:
  n = len(x)
  x = (x - x.mean()) * np.hanning(n)
  f = np.fft.rfftfreq(n, 1 / FS)
  P = np.abs(np.fft.rfft(x)) ** 2
  m = (f >= BAND[0]) & (f <= BAND[1])
  return float(1 / f[m][np.argmax(P[m])]) if m.any() else float("nan")


def stretches(d: dict[str, np.ndarray], min_mph: float, min_s: float) -> list[np.ndarray]:
  ok = (d["lat_active"] > 0) & (d["pressed"] == 0) & (d["blinker"] == 0) & (d["v"] > min_mph / 2.237)
  idx = np.where(ok)[0]
  if not len(idx):
    return []
  runs = np.split(idx, np.where((np.diff(idx) > 1) | (np.diff(d["t"][idx]) > 0.2))[0] + 1)
  return [r for r in runs if len(r) >= min_s * FS]


def analyze(d: dict[str, np.ndarray], r: np.ndarray) -> dict[str, float]:
  des, act = bandpass(d["des_la"][r]), bandpass(d["act_la"][r])
  lanes = (d["l_p"][r] > 0.5) & (d["r_p"][r] > 0.5) & np.isfinite(d["l_y"][r])
  wander = float("nan")
  if lanes.mean() > 0.8:  # lane position only when the lines are visible nearly the whole time
    offset = -(d["l_y"][r] + d["r_y"][r]) / 2
    offset = np.interp(np.arange(len(r)), np.where(lanes)[0], offset[lanes])
    wander = float(np.std(bandpass(offset)) * 100)
  cc = np.correlate(act, des, mode="full")
  lags = (np.arange(len(cc)) - (len(des) - 1)) / FS
  w = np.abs(lags) < 3
  return {"seg": d["seg"][r[0]], "minutes": len(r) / FS / 60, "mph": d["v"][r].mean() * 2.237, "wander": wander,
          "period": peak_period(d["des_la"][r]), "desired": float(np.std(des)), "error": float(np.std(des - act)),
          "follow": float(np.std(des - act) / max(np.std(des), 1e-6)), "lag": float(lags[w][np.argmax(cc[w])])}


def ab_split(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...]) -> list[tuple[str, np.ndarray]]:
  """Cut each stretch where the A/B test switches arm; keep pieces of at least min_s."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import ab_arm
  out = []
  for r in runs:
    arm = np.array([ab_arm(t, arms) for t in d["t"][r]])
    for piece in np.split(r, np.where(arm[1:] != arm[:-1])[0] + 1):
      if len(piece) >= min_s * FS:
        out.append((ab_arm(d["t"][piece[0]], arms), piece))
  return out


def ab_report(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...]) -> None:
  res: dict[str, list[dict]] = {a: [] for a in arms}
  print(f"{'arm':>5} {'seg':>4} {'min':>4} {'mph':>4} {'wander':>7} {'period':>7} {'follow':>7}")
  for arm, r in ab_split(d, runs, min_s, arms):
    x = analyze(d, r)
    res[arm].append(x)
    wander = f"{x['wander']:7.1f}" if np.isfinite(x["wander"]) else f"{'-':>7}"
    print(f"{arm:>5} {int(x['seg']):>4} {x['minutes']:>4.1f} {x['mph']:>4.0f} {wander} {x['period']:>7.1f} {x['follow']:>7.2f}")
  print()
  for arm in arms:
    w = np.array([x["wander"] for x in res[arm] if np.isfinite(x["wander"])])
    mins = sum(x["minutes"] for x in res[arm])
    mean = w.mean() if len(w) else np.nan
    se = w.std() / np.sqrt(len(w)) if len(w) > 1 else np.nan
    print(f"{arm:>5}: {mins:4.1f} min in {len(res[arm])} pieces, wander {mean:.1f} ± {se:.1f} cm rms")


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir")
  ap.add_argument("--min-mph", type=float, default=25)
  ap.add_argument("--min-s", type=float, default=45)
  ap.add_argument("--ab", choices=("deadband", "nnlc"), help="compare the arms of an A/B drive: SteerDeadbandCompAB or NnlcAB")
  args = ap.parse_args()

  d = load(args.route_dir)
  runs = stretches(d, args.min_mph, args.min_s)
  if args.ab:
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import DEADBAND_ARMS, NNLC_ARMS
    return ab_report(d, runs, args.min_s, DEADBAND_ARMS if args.ab == "deadband" else NNLC_ARMS)
  total = len(d["t"]) / FS / 60
  alone = sum(len(r) for r in runs) / FS / 60
  print(f"{total:.0f} min logged; {len(runs)} stretches of openpilot steering alone >= {args.min_s:.0f} s "
        + f"above {args.min_mph:.0f} mph, {alone:.1f} min\n")
  if not runs:
    return
  print(f"{'seg':>4} {'min':>4} {'mph':>4} {'wander':>7} {'period':>7} {'desired':>8} {'error':>6} {'follow':>7} {'lag':>5}")
  print(f"{'':>4} {'':>4} {'':>4} {'cm rms':>7} {'s':>7} {'m/s2':>8} {'m/s2':>6} {'':>7} {'s':>5}")
  res = [analyze(d, r) for r in runs]
  for x in res:
    wander = f"{x['wander']:7.1f}" if np.isfinite(x["wander"]) else f"{'-':>7}"
    print(f"{int(x['seg']):>4} {x['minutes']:>4.1f} {x['mph']:>4.0f} {wander} {x['period']:>7.1f} {x['desired']:>8.3f} "
          + f"{x['error']:>6.3f} {x['follow']:>7.2f} {x['lag']:>5.2f}")
  wt = np.array([x["minutes"] for x in res])
  follow = float(np.average([x["follow"] for x in res], weights=wt))
  print(f"\nweighted follow ratio {follow:.2f}: " + (
    "the controller follows the request; the swing is in what openpilot asks for (model/planner, camera offset, steer delay)"
    if follow < 0.3 else
    "the actual path misses the request; look at the controller and the steering (deadband, friction, saturation, delay)"))


if __name__ == "__main__":
  main()
