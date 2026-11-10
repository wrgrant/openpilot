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

  presses  (--ab only) how often the driver put torque on the wheel, per 10 minutes of openpilot steering
  shake    (--ab only) body lateral accel swing at 0.5-3 Hz, over all steering time, wheel presses included. Read
           this first: an arm that shakes the wheel leaves few press-free stretches, so its wander goes unscored
  sway     body lateral accel swing (v x yaw rate, m/s^2 rms). It does not use the lane lines, so it cross-checks
           wander: a change that only makes the van chase the model's lane estimate lowers wander but not sway

--ab deadband|nnlc|wander|nnlc_model|lane scores an A/B drive: it splits the stretches at the arm switches and compares
the arms. Give several route directories to pool drives; one half-hour drive yields only 3-5 scored blocks per arm,
and blocks differ by about 3 cm rms from road alone, so a 2 cm effect needs about 25 blocks per arm. Each route's
initData says which A/B tests were on: a route without the requested test is skipped (--force scores it anyway, as a
null test). --ab nnlc_model also scores signed curve run-wide and body lateral jerk against the locked VSIM forecast,
on lane-correction-off blocks only when OdysseyLaneCenterAB ran too, since the forecast was made without it.
"""
import argparse
import glob
import json
import os
from pathlib import Path

import numpy as np

FS = 20.0
BAND = (1 / 20, 1 / 7)  # Hz: 7-20 s periods
SHAKE_BAND = (0.5, 3.0)  # Hz: the fast side-to-side shake a driver feels as a jerky wheel
SHAKE_WINDOW_S, SHAKE_SETTLE_S = 5.0, 5.0
AB_FLAGS = {"deadband": "SteerDeadbandCompAB", "nnlc": "NnlcAB", "wander": "OdysseyWanderAB", "nnlc_model": "NnlcModelAB",
            "lane": "OdysseyLaneCenterAB"}


def load(route_dir: str) -> dict[str, np.ndarray]:
  from openpilot.tools.lib.logreader import LogReader
  cols = ("t", "seg", "v", "lat_active", "pressed", "blinker", "des_la", "act_la", "l_y", "r_y", "l_p", "r_p", "body")
  rows, lane_rows = [], []
  ab_params: dict[str, bool] | None = None
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
      if w == "initData" and ab_params is None:
        on = {e.key for e in m.initData.params.entries if e.value == b"1"}
        ab_params = {flag: flag in on for flag in AB_FLAGS.values()}
      elif w == "carState":
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
  return {**{c: data[:, i] for i, c in enumerate(cols)}, "lanes": np.array(lane_rows), "ab_params": ab_params or {}}


def bandpass(x: np.ndarray, band: tuple[float, float] = BAND) -> np.ndarray:
  """Zero-phase band-pass by FFT mask, after removing the linear trend."""
  n = len(x)
  x = x - np.polyval(np.polyfit(np.arange(n), x, 1), np.arange(n))
  f = np.fft.rfftfreq(n, 1 / FS)
  X = np.fft.rfft(x)
  X[(f < band[0]) | (f > band[1])] = 0
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
  body = d["body"][r] if "body" in d else np.full(len(r), np.nan)
  sway = float(np.std(bandpass(body))) if np.isfinite(body).all() else float("nan")
  cc = np.correlate(act, des, mode="full")
  lags = (np.arange(len(cc)) - (len(des) - 1)) / FS
  w = np.abs(lags) < 3
  return {"seg": d["seg"][r[0]], "minutes": len(r) / FS / 60, "mph": d["v"][r].mean() * 2.237, "wander": wander, "sway": sway,
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


def forecast_report(pieces: list[tuple[str, dict, np.ndarray]], block_s: float) -> None:
  path = Path(__file__).resolve().parent / "odyssey_sim/reports/vsim3/predictions.json"
  forecast = json.loads(path.read_text())
  if not forecast["locked"] or forecast["primary_forecast_version"] != 2:
    raise ValueError("expected the locked primary v2 forecast")
  blocks: dict[str, dict[tuple[int, int], list[dict]]] = {arm: {} for arm in ("stock", "S3", "S5")}
  for arm, d, r in pieces:
    block = (id(d), int(d["t"][r[0]] // block_s))  # block numbers repeat across drives
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


def summarize_pieces(pieces: list[tuple[str, dict, np.ndarray]], arms: tuple[str, ...], block_s: float | None) -> None:
  """Per-piece table, per-arm summary and each arm's difference from arms[0]. A block is the unit: several pieces of one
  block are averaged, because they are not independent."""
  per_block: dict[str, dict[tuple[int, int], list[dict]]] = {a: {} for a in arms}
  minutes = dict.fromkeys(arms, 0.0)
  print(f"{'arm':>7} {'seg':>4} {'min':>4} {'mph':>4} {'wander':>7} {'sway':>6} {'period':>7} {'follow':>7}")
  for arm, d, r in pieces:
    x = analyze(d, r)
    per_block[arm].setdefault((id(d), int(d["t"][r[0]] // (block_s or 1.0))), []).append(x)
    minutes[arm] += x["minutes"]
    wander = f"{x['wander']:7.1f}" if np.isfinite(x["wander"]) else f"{'-':>7}"
    sway = f"{x['sway']:6.3f}" if np.isfinite(x["sway"]) else f"{'-':>6}"
    print(f"{arm:>7} {int(x['seg']):>4} {x['minutes']:>4.1f} {x['mph']:>4.0f} {wander} {sway} {x['period']:>7.1f} {x['follow']:>7.2f}")
  print()
  values: dict[str, dict[str, np.ndarray]] = {}
  for arm in arms:
    values[arm] = {}
    for metric in ("wander", "sway"):
      per = [[x[metric] for x in xs if np.isfinite(x[metric])] for xs in per_block[arm].values()]
      values[arm][metric] = np.array([np.mean(v) for v in per if v])
    w, sw = values[arm]["wander"], values[arm]["sway"]
    se = w.std(ddof=1) / np.sqrt(len(w)) if len(w) > 1 else np.nan
    print(f"{arm:>7}: {minutes[arm]:4.1f} min, {len(per_block[arm])} blocks ({len(w)} with lane lines), "
          + f"wander {w.mean() if len(w) else np.nan:.1f} ± {se:.1f} cm rms, sway {sw.mean() if len(sw) else np.nan:.3f} m/s2")
  for arm in arms[1:]:
    for metric, unit in (("wander", "cm rms"), ("sway", "m/s2")):
      delta, ci = block_difference(values[arm][metric], values[arms[0]][metric])
      interval = f"[{ci[0]:+.2f}, {ci[1]:+.2f}]" if ci is not None else "[too few blocks for an interval]"
      print(f"{arm} - {arms[0]} {metric}: {delta:+.2f} {interval} {unit}")
  if arms == ("stock", "S3", "S5"):
    forecast_report(pieces, block_s or 180.0)


def nudges(d: dict[str, np.ndarray], arms: tuple[str, ...], block_s: float, settle_s: float, min_mph: float) -> dict[str, tuple[int, float]]:
  """Per arm: how often the driver put torque on the wheel while openpilot was steering with no blinker above min_mph,
  and the minutes that covers. A press counts once: touches less than 2 s apart are one press."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import ab_arm
  ok = (d["lat_active"] > 0) & (d["blinker"] == 0) & (d["v"] > min_mph / 2.237) & ((d["t"] % block_s) >= settle_s)
  on = d["pressed"] > 0
  recent = np.convolve(on, np.ones(int(2 * FS)), mode="full")[:len(on)] - on  # presses in the 2 s before each frame
  starts = ok & on & (recent == 0)
  arm = np.array([ab_arm(t, arms, block_s) for t in d["t"]])
  return {a: (int(starts[arm == a].sum()), float(ok[arm == a].sum() / FS / 60)) for a in arms}


def shake(d: dict[str, np.ndarray], arms: tuple[str, ...], block_s: float, min_mph: float) -> dict[str, list[float]]:
  """Per arm, one value per A/B block: the median over 5 s windows of the body's lateral accel swing at 0.5-3 Hz
  (m/s2 rms). Every window where openpilot steers above min_mph with no blinker counts, wheel presses included: a
  controller that shakes the wheel trips the press sensor, so scoring only press-free stretches hides the arm that
  shakes (drives 1ac-1ae: S3 and S5 shook the van four times as hard as stock and scored as "too little data")."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import ab_arm
  n = int(SHAKE_WINDOW_S * FS)
  ok = (d["lat_active"] > 0) & (d["blinker"] == 0) & (d["v"] > min_mph / 2.237) & np.isfinite(d["body"])
  per_block: dict[tuple[str, int], list[float]] = {}
  for a in range(0, len(ok) - n, n):
    t0, t1 = d["t"][a], d["t"][a + n - 1]
    if ok[a:a + n].all() and t1 - t0 < SHAKE_WINDOW_S + 0.3 and t0 // block_s == t1 // block_s and t0 % block_s >= SHAKE_SETTLE_S:
      per_block.setdefault((ab_arm(t0, arms, block_s), int(t0 // block_s)), []).append(float(np.std(bandpass(d["body"][a:a + n], SHAKE_BAND))))
  out: dict[str, list[float]] = {arm: [] for arm in arms}
  for (arm, _), values in sorted(per_block.items(), key=lambda kv: kv[0][1]):
    if len(values) >= 3:  # at least 15 s of the block
      out[arm].append(float(np.median(values)))
  return out


def shake_report(shakes: dict[str, list[float]], arms: tuple[str, ...]) -> None:
  print("fast shake, 0.5-3 Hz body lateral accel, all steering time including wheel presses (m/s2 rms; 0.035 is this "
        + "van's normal, and S5 at 0.08-0.11 felt very jerky):")
  print("  " + ", ".join(f"{a} {np.mean(shakes[a]) if shakes[a] else float('nan'):.3f} ({len(shakes[a])} blocks)" for a in arms))
  for arm in arms[1:]:
    delta, ci = block_difference(np.array(shakes[arm]), np.array(shakes[arms[0]]))
    interval = f"[{ci[0]:+.3f}, {ci[1]:+.3f}]" if ci is not None else "[too few blocks for an interval]"
    print(f"  {arm} - {arms[0]} shake: {delta:+.3f} {interval} m/s2")


def ab_pieces(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...],
              block_s: float | None = None) -> list[tuple[str, dict, np.ndarray]]:
  settle = 30.0 if block_s and block_s >= 120 else 0.0  # long-block tests skip each block's first 30 s
  return [(arm, d, r) for arm, r in ab_split(d, runs, min_s, arms, block_s, settle)]


def ab_report(d: dict[str, np.ndarray], runs: list[np.ndarray], min_s: float, arms: tuple[str, ...], block_s: float | None = None) -> None:
  summarize_pieces(ab_pieces(d, runs, min_s, arms, block_s), arms, block_s)


def ab_report_routes(paths: list[str], routes: list[dict], name: str, min_mph: float, min_s: float, force: bool = False) -> None:
  """Pool an A/B test over drives, using each drive's own record of which tests were on."""
  from openpilot.sunnypilot.selfdrive.controls.lib.odyssey_ab import (AB_BLOCK_S, DEADBAND_ARMS, LANE_ARMS, NNLC_ARMS,
                                                                      NNLC_MODEL_ARMS, WANDER_ARMS, WANDER_BLOCK_S, ab_arm)
  tests = {"deadband": (DEADBAND_ARMS, AB_BLOCK_S), "nnlc": (NNLC_ARMS, AB_BLOCK_S), "wander": (WANDER_ARMS, WANDER_BLOCK_S),
           "nnlc_model": (NNLC_MODEL_ARMS, WANDER_BLOCK_S), "lane": (LANE_ARMS, WANDER_BLOCK_S)}
  arms, block = tests[name]
  pieces: list[tuple[str, dict, np.ndarray]] = []
  pressed = {a: [0, 0.0] for a in arms}
  shakes: dict[str, list[float]] = {a: [] for a in arms}
  for path, d in zip(paths, routes, strict=True):
    on = {k for k, flag in AB_FLAGS.items() if d["ab_params"].get(flag)}
    label = os.path.basename(os.path.normpath(path))
    if name not in on and not force:
      print(f"{label}: skipped, {AB_FLAGS[name]} was not on (on: {', '.join(sorted(on)) or 'none'})")
      continue
    got = ab_pieces(d, stretches(d, min_mph, min_s), min_s, arms, block)
    note = "" if name in on else " (forced: a null test, the arms are only time blocks)"
    if name == "nnlc_model" and "lane" in on:
      kept = [p for p in got if ab_arm(d["t"][p[2][0]], LANE_ARMS, WANDER_BLOCK_S) == "off"]
      note += f"; kept {len(kept)} of {len(got)} pieces, those with the lane correction off"
      got = kept
    elif name == "lane" and "nnlc_model" in on:
      cells = {(m, a): 0 for m in NNLC_MODEL_ARMS for a in arms}
      for arm, _, r in got:
        cells[(ab_arm(d["t"][r[0]], NNLC_MODEL_ARMS, WANDER_BLOCK_S), arm)] += 1
      note += "; NNLC model also rotated, pieces per model off/on: " + ", ".join(
        f"{m} {cells[(m, 'off')]}/{cells[(m, 'on')]}" for m in NNLC_MODEL_ARMS)
    elif on - {name}:
      note += f"; also rotating: {', '.join(sorted(on - {name}))}"
    print(f"{label}: {len(got)} pieces{note}")
    pieces += got
    for a, (count, minutes) in nudges(d, arms, block, 30.0 if block >= 120 else 0.0, min_mph).items():
      pressed[a][0] += count
      pressed[a][1] += minutes
    for a, values in shake(d, arms, block, min_mph).items():
      shakes[a] += values
  print()
  summarize_pieces(pieces, arms, block)
  shake_report(shakes, arms)
  print("driver hands on the wheel (presses per 10 min of openpilot steering, no blinker): "
        + ", ".join(f"{a} {10 * n / m if m else float('nan'):.1f} ({n} in {m:.0f} min)" for a, (n, m) in pressed.items()))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir", nargs="+")
  ap.add_argument("--min-mph", type=float, default=25)
  ap.add_argument("--min-s", type=float, default=45)
  ap.add_argument("--ab", choices=("deadband", "nnlc", "wander", "nnlc_model", "lane"),
                  help="compare the arms of an A/B test, pooled over the given drives")
  ap.add_argument("--force", action="store_true", help="score drives where the A/B test was not on, as a null test")
  args = ap.parse_args()

  routes = [load(p) for p in args.route_dir]
  if args.ab:
    return ab_report_routes(args.route_dir, routes, args.ab, args.min_mph, args.min_s, args.force)
  for path, d in zip(args.route_dir, routes, strict=True):
    if len(routes) > 1:
      print(f"\n== {os.path.basename(os.path.normpath(path))}")
    wander_report(d, args.min_mph, args.min_s)


def wander_report(d: dict[str, np.ndarray], min_mph: float, min_s: float) -> None:
  runs = stretches(d, min_mph, min_s)
  total = len(d["t"]) / FS / 60
  alone = sum(len(r) for r in runs) / FS / 60
  print(f"{total:.0f} min logged; {len(runs)} stretches of openpilot steering alone >= {min_s:.0f} s "
        + f"above {min_mph:.0f} mph, {alone:.1f} min\n")
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
