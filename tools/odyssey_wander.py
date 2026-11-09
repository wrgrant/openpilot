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
compares the arms. --ab nnlc_model also scores signed curve run-wide and body lateral jerk against the locked VSIM forecast.
"""
import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np

FS = 20.0
BAND = (1 / 20, 1 / 7)  # Hz: 7-20 s periods


def load(route_dir: str) -> dict[str, np.ndarray]:
  from openpilot.tools.lib.logreader import LogReader
  cols = ("t", "seg", "v", "lat_active", "pressed", "blinker", "des_la", "act_la", "l_y", "r_y", "l_p", "r_p", "body")
  rows, lane_rows = [], []
  segs = sorted(glob.glob(os.path.join(route_dir, "*--*")), key=lambda d: int(d.rsplit("--", 1)[1]))
  for d in segs:
    seg = int(d.rsplit("--", 1)[1])
    st = dict.fromkeys(cols, 0.0)
    st.update(seg=seg, l_y=np.nan, r_y=np.nan, body=np.nan)
    lanes = np.full((2, 33), np.nan)
    yaw_rate = np.nan
    last_t = -1.0
    for m in sorted(LogReader(os.path.join(d, "rlog.zst")), key=lambda m: m.logMonoTime):
      w = m.which()
      if w == "carState":
        cs = m.carState
        st["body"] = cs.vEgo * yaw_rate
        st.update(v=cs.vEgo, pressed=float(cs.steeringPressed), blinker=float(cs.leftBlinker or cs.rightBlinker))
        t = m.logMonoTime / 1e9
        if t - last_t >= 0.0495:  # 20 Hz rows, stamped on carState
          st["t"] = last_t = t
          rows.append([st[c] for c in cols])
          lane_rows.append(lanes)
      elif w == "carControl":
        st["lat_active"] = float(m.carControl.latActive)
        angular = m.carControl.angularVelocity
        yaw_rate = float(angular[2]) if len(angular) >= 3 else np.nan
      elif w == "controlsState":
        lcs = m.controlsState.lateralControlState
        if lcs.which() == "torqueState":
          st.update(des_la=lcs.torqueState.desiredLateralAccel, act_la=lcs.torqueState.actualLateralAccel)
      elif w == "modelV2" and len(m.modelV2.laneLines) == 4:
        lanes = np.array([m.modelV2.laneLines[i].y for i in (1, 2)])
        st.update(l_y=m.modelV2.laneLines[1].y[0], r_y=m.modelV2.laneLines[2].y[0],
                  l_p=m.modelV2.laneLineProbs[1], r_p=m.modelV2.laneLineProbs[2])
  data = np.array(rows, dtype=float)
  return {**{c: data[:, i] for i, c in enumerate(cols)}, "lanes": np.array(lane_rows)}


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


def ab_split(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...],
             block_s: float | None = None, settle_s: float = 0.0) -> list[tuple[str, np.ndarray]]:
  """Cut each stretch where the A/B test switches arm, dropping the first settle_s of each block; keep pieces of at
  least min_s."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import AB_BLOCK_S, ab_arm
  block_s = AB_BLOCK_S if block_s is None else block_s
  out = []
  for r in runs:
    r = r[(d["t"][r] % block_s) >= settle_s]
    for rr in np.split(r, np.where(np.diff(r) > 1)[0] + 1) if len(r) else []:
      arm = np.array([ab_arm(t, arms, block_s) for t in d["t"][rr]])
      for piece in np.split(rr, np.where(arm[1:] != arm[:-1])[0] + 1):
        if len(piece) >= min_s * FS:
          out.append((ab_arm(d["t"][piece[0]], arms, block_s), piece))
  return out


def forecast_piece(d: dict[str, np.ndarray], r: np.ndarray) -> dict:
  """Adapt a settled autonomous piece to the unchanged, locked VSIM metric."""
  from tools.odyssey_sim.vsim.metrics import curve_scores

  t = d["t"][r] - d["t"][r[0]]
  lanes = d["lanes"][r]
  finite = np.isfinite(lanes).all(axis=(1, 2))
  confidence = finite & (d["l_p"][r] > 0.5) & (d["r_p"][r] > 0.5)
  # Unsupported geometry must not enter the polynomial fit; its visibility stays false.
  lanes = np.where(np.isfinite(lanes), lanes, 0.0)
  rows = [{"t": at, "speed": v, "lane": lane, "logged_lane": lane}
          for at, v, lane in zip(t, d["v"][r], lanes, strict=True)]
  native = {"t": t, "des": d["des_la"][r], "act": d["act_la"][r], "body": d["body"][r], "cmd": np.zeros(len(r))}
  motion = [{"t": at, "body": body} for at, body in zip(t, native["body"], strict=True)]
  score = curve_scores(rows, motion, native, 0, confidence)
  return {"widths": [ev["logged_run_wide_m"] for ev in score["events"] if ev["logged_run_wide_m"] is not None],
          "jerk": score["logged_body_jerk_rms_mps3"], "jerk_n": len(np.arange(t[0], t[-1], 1 / FS))}


def block_difference(candidate: np.ndarray, stock: np.ndarray) -> tuple[float, np.ndarray | None]:
  """Independent-arm block bootstrap: road arms occupy different, unpaired blocks."""
  if not len(candidate) or not len(stock):
    return np.nan, None
  difference = float(candidate.mean() - stock.mean())
  if min(len(candidate), len(stock)) < 2:
    return difference, None
  rng = np.random.default_rng(0)
  samples = (rng.choice(candidate, (10000, len(candidate))).mean(axis=1)
             - rng.choice(stock, (10000, len(stock))).mean(axis=1))
  return difference, np.percentile(samples, [2.5, 97.5])


def forecast_report(d: dict[str, np.ndarray], pieces: list[tuple[str, np.ndarray]], block_s: float) -> None:
  path = Path(__file__).resolve().parent / "odyssey_sim/reports/vsim3/predictions.json"
  forecast = json.loads(path.read_text())
  if not forecast["locked"] or forecast["primary_forecast_version"] != 2:
    raise ValueError("expected the locked primary v2 forecast")
  blocks: dict[str, dict[int, list[dict]]] = {arm: {} for arm in ("stock", "S3", "S5")}
  for arm, r in pieces:
    block = int(d["t"][r[0]] // block_s)
    score = forecast_piece(d, r)
    score["wander"] = analyze(d, r)["wander"]
    blocks[arm].setdefault(block, []).append(score)
  metrics = {arm: {metric: [] for metric in forecast["units"]} for arm in blocks}
  for arm, arm_blocks in blocks.items():
    for scores in arm_blocks.values():
      widths = [w for x in scores for w in x["widths"]]
      wander = [x["wander"] for x in scores if np.isfinite(x["wander"])]
      # A missing body piece makes this block unavailable; never rank only its surviving pieces.
      jerk_ok = all(x["jerk"] is not None and np.isfinite(x["jerk"]) for x in scores)
      values = {
        "wander_cm": float(np.mean(wander)) if wander else np.nan,
        "curve_run_wide_m": float(np.median(widths)) if widths else np.nan,
        "body_jerk_rms_mps3": (float(np.sqrt(np.average([x["jerk"] ** 2 for x in scores],
                                                                      weights=[x["jerk_n"] for x in scores]))) if jerk_ok else np.nan),
      }
      for metric, value in values.items():
        if np.isfinite(value):
          metrics[arm][metric].append(value)
  print(f"\nLocked forecast v2: {path}")
  print("Road estimator: equal-weight block means; curve = median visible events per block, jerk = pooled 20 Hz derivative RMS per block.")
  print("95% CI: 10000 seeded independent-arm block bootstrap resamples; at least two blocks per arm required.")
  print(forecast["interval_caution"])
  print("Arm labels are inferred from monotonic time; this report does not verify NnlcModelAB was enabled.")
  for metric, unit in forecast["units"].items():
    for arm in blocks:
      values = np.array(metrics[arm][metric])
      mean = f"{values.mean():+.6f}" if len(values) else "unavailable"
      print(f"{arm} {metric}: {mean} {unit} ({len(values)} blocks)")
    stock = np.array(metrics["stock"][metric])
    for arm in ("S3", "S5"):
      delta, ci = block_difference(np.array(metrics[arm][metric]), stock)
      observed = f"{delta:+.6f}" if np.isfinite(delta) else "unavailable"
      interval = f"[{ci[0]:+.6f}, {ci[1]:+.6f}]" if ci is not None else "[CI unavailable]"
      pred = forecast["predictions"][arm][metric]
      if pred["mean"] is None or pred["ci95"] is None:
        predicted, inside = "unavailable", "unavailable"
      else:
        lo, hi = pred["ci95"]
        predicted = f"{pred['mean']:+.6f} [{lo:+.6f}, {hi:+.6f}]"
        inside = ("yes" if lo <= delta <= hi else "no") if np.isfinite(delta) else "unavailable"
      print(f"{arm} - stock {metric}: {observed} {interval} {unit}; "
            + f"predicted {predicted} {unit}; inside predicted interval: {inside}")


def ab_report(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...], block_s: float | None = None) -> None:
  res: dict[str, list[dict]] = {a: [] for a in arms}
  print(f"{'arm':>5} {'seg':>4} {'min':>4} {'mph':>4} {'wander':>7} {'period':>7} {'follow':>7}")
  settle = 30.0 if block_s and block_s >= 120 else 0.0  # long-block tests skip each block's first 30 s
  pieces = ab_split(d, runs, min_s, arms, block_s, settle)
  for arm, r in pieces:
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

  if arms == ("stock", "S3", "S5"):
    forecast_report(d, pieces, block_s or 180.0)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir")
  ap.add_argument("--min-mph", type=float, default=25)
  ap.add_argument("--min-s", type=float, default=45)
  ap.add_argument("--ab", choices=("deadband", "nnlc", "wander", "nnlc_model", "lane"),
                  help="compare the arms of an A/B drive: SteerDeadbandCompAB, NnlcAB, OdysseyWanderAB or NnlcModelAB")
  args = ap.parse_args()

  d = load(args.route_dir)
  runs = stretches(d, args.min_mph, args.min_s)
  if args.ab:
    from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import (AB_BLOCK_S, DEADBAND_ARMS, NNLC_ARMS, NNLC_MODEL_ARMS,
                                                                        WANDER_ARMS, WANDER_BLOCK_S, LANE_ARMS)
    arms, block = {"deadband": (DEADBAND_ARMS, AB_BLOCK_S), "nnlc": (NNLC_ARMS, AB_BLOCK_S),
                   "wander": (WANDER_ARMS, WANDER_BLOCK_S), "nnlc_model": (NNLC_MODEL_ARMS, WANDER_BLOCK_S), "lane": (LANE_ARMS, WANDER_BLOCK_S)}[args.ab]
    return ab_report(d, runs, args.min_s, arms, block)
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
