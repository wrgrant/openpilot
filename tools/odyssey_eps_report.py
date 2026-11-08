#!/usr/bin/env python3
"""Report on an Odyssey steering-characterization drive (openpilot/tools/lateral_maneuvers/odyssey_suite.py).

Decodes the van's own steering signals from the logged CAN bus at 100 Hz (openpilot's torque command, the EPS motor
torque, wheel angle and rate, driver torque, the VSA's lateral accel) and, for each maneuver run, measures:

The wheel angle on CAN reads in 0.1 degree steps, but the EPS also reports the wheel's rate in 0.1 deg/s steps, and that
rate moves before the angle ticks over (it comes from a finer sensor than the angle). fine_angle() integrates the rate
and keeps it inside the 0.1 degree reading, so the wheel's first ~0.02 degrees of motion are visible.

  delay      request change -> motor torque -> wheel angle -> van lateral accel
  breakaway  the command at which the wheel starts to move after each direction change, split left/right and
             away-from-straight / back-toward-straight (is the steering one-sided?)
  hysteresis for the slow sweeps, how much more command it takes to move the wheel one way than to hold it on the way
             back (the friction band the deadband kick has to clear)

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps_report.py ~/comma-logs/<dongle>_<route> [--plot out.png]
It also prints fitted values for tools/odyssey_lateral_sim.py (--deadband, --friction, --lag, --bias).
"""
import argparse
import glob
import os
from collections import defaultdict

import numpy as np

DBC = "honda_odyssey_exl_2018_generated"
STEER_MAX = 4096
COLS = ("t", "v", "angle", "rate", "drv_tq", "motor_tq", "cmd_tq", "vsa_lat", "req_la", "model_la", "maneuver", "pressed")
MOVED_DEG = 0.02  # wheel motion that counts as "moving", on the fine angle


def load(route_dir: str) -> tuple[dict[str, np.ndarray], list[str]]:
  from opendbc.can import CANParser
  from openpilot.tools.lib.logreader import LogReader
  rows, names = [], []
  for d in sorted(glob.glob(os.path.join(route_dir, "*--*")), key=lambda d: int(d.rsplit("--", 1)[1])):
    car = CANParser(DBC, [("STEERING_SENSORS", 0), ("STEER_STATUS", 0), ("STEER_MOTOR_TORQUE", 0), ("VEHICLE_DYNAMICS", 0)], 0)
    cmd = CANParser(DBC, [("STEERING_CONTROL", 0)], 0)
    st = dict.fromkeys(COLS, 0.0)
    st["maneuver"] = -1
    text = ""
    for m in sorted(LogReader(os.path.join(d, "rlog.zst")), key=lambda m: m.logMonoTime):
      w = m.which()
      if w == "can":
        up = car.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.can])])
        if up:
          vl = car.vl
          st.update(angle=vl["STEERING_SENSORS"]["STEER_ANGLE"], rate=vl["STEER_STATUS"]["STEER_ANGLE_RATE"],
                    drv_tq=vl["STEER_STATUS"]["STEER_TORQUE_SENSOR"], motor_tq=vl["STEER_MOTOR_TORQUE"]["MOTOR_TORQUE"],
                    vsa_lat=vl["VEHICLE_DYNAMICS"]["LAT_ACCEL"])
          if 342 in up:
            st["t"] = m.logMonoTime / 1e9
            rows.append([st[c] for c in COLS])
      elif w == "sendcan":
        if cmd.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.sendcan])]):
          st["cmd_tq"] = cmd.vl["STEERING_CONTROL"]["STEER_TORQUE"]
      elif w == "carState":
        st.update(v=m.carState.vEgo, pressed=float(m.carState.steeringPressed))
      elif w == "modelV2":
        st["model_la"] = m.modelV2.action.desiredCurvature * max(st["v"], 1.0) ** 2
      elif w == "alertDebug":
        text = m.alertDebug.alertText2
      elif w == "lateralManeuverPlan":
        if m.valid and text:
          if text not in names:
            names.append(text)
          st["maneuver"] = names.index(text)
          st["req_la"] = m.lateralManeuverPlan.desiredCurvature * max(st["v"], 1.0) ** 2
        else:
          st["maneuver"], st["req_la"] = -1, 0.0
  data = np.array(rows, dtype=float)
  d = {c: data[:, i] for i, c in enumerate(COLS)}
  d["fine"] = fine_angle(d["angle"], d["rate"])
  # the suite rides on lane keeping: the maneuver is the plan minus what the model asked for
  d["man_la"] = np.where(d["maneuver"] >= 0, d["req_la"] - d["model_la"], 0.0)
  return d, names


def fine_angle(angle: np.ndarray, rate: np.ndarray, dt: float = 0.01) -> np.ndarray:
  """Wheel angle finer than the 0.1 degree reading: integrate the EPS rate, held inside the reading's step."""
  out = np.empty_like(angle)
  x = angle[0]
  for i in range(len(angle)):
    x = min(max(x + rate[i] * dt, angle[i] - 0.05), angle[i] + 0.05)
    out[i] = x
  return out


def runs(d: dict[str, np.ndarray]) -> list[np.ndarray]:
  idx = np.where(d["maneuver"] >= 0)[0]
  if not len(idx):
    return []
  split = np.where((np.diff(idx) > 5) | (np.diff(d["maneuver"][idx]) != 0))[0] + 1
  return [r for r in np.split(idx, split) if len(r) > 300]


def first_after(mask: np.ndarray, start: int) -> int | None:
  hits = np.where(mask[start:])[0]
  return int(start + hits[0]) if len(hits) else None


def reversed_by(x: np.ndarray, direction: float, amount: float) -> np.ndarray:
  """True from where x has moved `amount` in `direction` back from the furthest it got the other way."""
  back = np.minimum.accumulate(direction * x)
  return direction * x - back >= amount - 1e-6


def analyze_run(d: dict[str, np.ndarray], r: np.ndarray, center: float) -> dict:
  fine = "fine" in d
  t, cmd = d["t"][r], -d["cmd_tq"][r]  # -cmd: + = toward +angle (left)
  req = d["man_la"][r] if "man_la" in d else d["req_la"][r]
  ang = d["fine"][r] if fine else d["angle"][r]
  moved = MOVED_DEG if fine else 0.1
  mot, lat = d["motor_tq"][r], d["vsa_lat"][r]
  out = {"delays": [], "breakaway": [], "mph": float(np.median(d["v"][r]) * 2.237) if "v" in d else float("nan")}
  # direction changes of the request (the steps and the slow crossings)
  sgn = np.sign(np.where(np.abs(req) > 0.01, req, 0))
  changes = [i for i in range(1, len(sgn)) if sgn[i] != 0 and sgn[i] != sgn[i - 1]]
  for i0 in changes:
    direction = sgn[i0]
    i_mot = first_after((np.sign(cmd) == direction) & (mot >= 3), i0)
    # the wheel (and van) may coast on the old way first: measure the turn-around from the furthest point reached
    i_ang = first_after(np.r_[np.zeros(i0, bool), reversed_by(ang[i0:], direction, moved)], i0)
    i_lat = first_after(np.r_[np.zeros(i0, bool), reversed_by(lat[i0:], direction, 0.03)], i0)
    out["delays"].append([(i - i0) / 100 if i is not None else np.nan for i in (i_mot, i_ang, i_lat)])
    if i_ang is not None:
      off = ang[i_ang] - center
      kind = "toward straight" if (abs(off) > 0.15 and np.sign(off) != direction) else ("away from straight" if abs(off) > 0.15 else "at straight")
      out["breakaway"].append((direction, kind, direction * cmd[i_ang] / STEER_MAX))
  # hysteresis: command needed to move the wheel one way vs the other at the same angle (slow sweeps)
  moving = np.abs(np.gradient(ang) * 100) >= 0.3
  up, dn = moving & (np.gradient(ang) > 0), moving & (np.gradient(ang) < 0)
  if up.sum() > 20 and dn.sum() > 20:
    out["hysteresis"] = float(np.median(cmd[up]) - np.median(cmd[dn])) / STEER_MAX
  out["peak_motor_lag"] = float(t[np.argmax(np.abs(mot))] - t[np.argmax(np.abs(req))]) if np.abs(req).max() > 0 else np.nan
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir")
  ap.add_argument("--plot", help="write a PNG of command vs wheel angle for each maneuver")
  args = ap.parse_args()
  d, names = load(args.route_dir)
  rs = runs(d)
  print(f"{len(rs)} maneuver runs across {len(names)} maneuvers")
  if not rs:
    print("no lateralManeuverPlan in this drive: was Lateral Maneuver Mode on with LateralManeuverSuite=odyssey?")
    return
  center = float(np.median(d["angle"][d["v"] > 20]))  # straight ahead: the median wheel angle at highway speed
  by_name: dict[str, list[dict]] = defaultdict(list)
  for r in rs:
    by_name[names[int(d["maneuver"][r[0]])]].append(analyze_run(d, r, center))

  all_delays, brk = [], defaultdict(list)
  print(f"\nstraight-ahead wheel angle at highway speed: {center:.2f} deg")
  print(f"{'maneuver':36s} {'runs':>4} {'mph':>4} {'motor':>6} {'wheel':>6} {'van':>6}  {'breakaway torque (fraction of max)':>34} {'hyst':>6}")
  for name, res in by_name.items():
    dl = np.array([x for r in res for x in r["delays"]]) if any(r["delays"] for r in res) else np.full((1, 3), np.nan)
    all_delays.append(dl)
    b = [x for r in res for x in r["breakaway"]]
    for x in b:
      brk[(x[0], x[1])].append(x[2])
    bs = f"{np.median([x[2] for x in b]):+.3f} (n={len(b)})" if b else "-"
    hy = [r["hysteresis"] for r in res if "hysteresis" in r]
    mph = np.median([r["mph"] for r in res])
    print(f"{name:36s} {len(res):4d} {mph:4.0f} {np.nanmedian(dl[:, 0]):6.2f} {np.nanmedian(dl[:, 1]):6.2f} {np.nanmedian(dl[:, 2]):6.2f}"
          + f"  {bs:>34} {np.median(hy) if hy else np.nan:6.3f}")
  print(f"  (delays in s from the request changing direction to: motor torque, wheel moving {MOVED_DEG} deg, van lateral accel 0.03)")

  print("\nbreakaway torque by direction (median fraction of max torque, n):")
  for (direction, kind), vals in sorted(brk.items()):
    print(f"  {'left ' if direction > 0 else 'right'} {kind:20s} {np.median(vals):+.3f}  (n={len(vals)})")
  away = [v for (dr, k), vals in brk.items() if k == "away from straight" for v in vals]
  toward = [v for (dr, k), vals in brk.items() if k == "toward straight" for v in vals]
  if away and toward:
    print(f"  one-sidedness: moving away from straight takes {np.median(away) - np.median(toward):+.3f} more torque than returning")

  dl = np.vstack(all_delays)
  hy_all = [r["hysteresis"] for res in by_name.values() for r in res if "hysteresis" in r]
  deadband = float(np.median([v for vals in brk.values() for v in vals])) if brk else float("nan")
  print("\nfor tools/odyssey_lateral_sim.py:")
  print(f"  --deadband {deadband:.3f}   (median breakaway torque)")
  print(f"  --lag {np.nanmedian(dl[:, 2]) - np.nanmedian(dl[:, 1]):.2f}   (wheel moving -> van responding)")
  if hy_all:
    print(f"  --friction {abs(np.median(hy_all)) * 1.8 / 2:.3f}   (half the hysteresis band, in m/s^2 at latAccelFactor 1.8)")
  if away and toward:
    print(f"  --bias {(np.median(away) - np.median(toward)) * 1.8 / 2:+.3f}   (one-sidedness, in m/s^2)")

  if args.plot:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(by_name), figsize=(4 * len(by_name), 4), squeeze=False)
    for ax, (name, _) in zip(axes[0], by_name.items(), strict=True):
      for r in rs:
        if names[int(d["maneuver"][r[0]])] == name:
          ax.plot(d["fine"][r] - center, -d["cmd_tq"][r] / STEER_MAX, lw=0.8)
      ax.set_title(name, fontsize=8)
      ax.set_xlabel("wheel angle from straight (deg)")
      ax.set_ylabel("command (fraction of max)")
    fig.tight_layout()
    fig.savefig(args.plot, dpi=110)
    print(f"\nplot: {args.plot}")


if __name__ == "__main__":
  main()
