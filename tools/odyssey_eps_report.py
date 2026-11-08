#!/usr/bin/env python3
"""Report on an Odyssey steering-characterization drive (openpilot/tools/lateral_maneuvers/odyssey_suite.py).

Decodes the van's own steering signals from the logged CAN bus at 100 Hz (openpilot's torque command, the EPS motor
torque, wheel angle and rate, driver torque, the VSA's lateral accel).

The wheel angle on CAN reads in 0.1 degree steps, but the EPS also reports the wheel's rate in 0.1 deg/s steps, and that
rate moves before the angle ticks over (it comes from a finer sensor than the angle). fine_angle() integrates the rate
and keeps it inside the 0.1 degree reading, so the wheel's first ~0.02 degrees of motion are visible.

For each held pulse (odyssey_suite), aligned on the maneuver's own schedule, it measures how far the wheel turns and
the van moves compared with holds that had no pulse, by pulse size, width and direction. The smallest pulse that does
anything is the deadband the SteerDeadbandComp kick has to clear; left vs right shows whether the steering is
one-sided.

  PYTHONPATH=$PWD .venv/bin/python tools/odyssey_eps_report.py ~/comma-logs/<dongle>_<route>
It also prints the deadband to give tools/odyssey_lateral_sim.py (--deadband).
"""
import argparse
import glob
import os
from collections import defaultdict

import numpy as np

DBC = "honda_odyssey_exl_2018_generated"
STEER_MAX = 4096
COLS = ("t", "v", "angle", "rate", "drv_tq", "motor_tq", "cmd_tq", "vsa_lat", "req_la", "model_la", "probe", "maneuver", "pressed")


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
          st["probe"] = m.lateralManeuverPlan.torqueOffset
        else:
          st["maneuver"], st["req_la"], st["probe"] = -1, 0.0, 0.0
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


def schedule(name: str):
  """The maneuver's request (lateral accel or torque) as a function of time from the run start, and its hold windows."""
  from openpilot.tools.lateral_maneuvers.odyssey_suite import maneuvers
  m = next((m for m in maneuvers() if m.description == name), None)
  if m is None:
    return None
  a = m.actions[0]
  return (lambda t: np.interp(t, a.time_bp, a.accel_bp)), m


PULSE_WINDOW_S = 1.5  # after each pulse starts


def held_pulses(d, by_name: dict[str, list[np.ndarray]]) -> list[dict]:
  """One row per pulse in the held-pulse maneuvers: wheel, motor and van response relative to the pulse start."""
  out = []
  for name, rs in by_name.items():
    if not name.startswith("held pulses"):
      continue
    sched = schedule(name)
    if sched is None:
      continue
    f, m = sched
    a = m.actions[0]
    width = float(name.split()[2].rstrip("s"))
    t = np.arange(0, a.time_bp[-1], 0.01)
    v = f(t)
    starts = [hold[0] for hold in m.holds]
    motor = d["motor_tq"] * np.sign(-d["cmd_tq"])  # the EPS reports magnitude only; + = left like the command
    pre = 0.3
    for r in rs:
      for h0 in starts:
        i_on = r[0] + int(round((h0 + pre) * 100))
        lvl = float(v[int(round((h0 + pre + width / 2) * 100))])
        n = int(PULSE_WINDOW_S * 100)
        if i_on + n > min(r[-1] + 1, len(d["t"])):
          continue
        sgn = 1.0 if lvl >= 0 else -1.0
        if abs(lvl) < 1e-6:  # the no-pulse holds alternate direction like the pulses do
          sgn = 1.0 if starts.index(h0) % 2 == 0 else -1.0
          sgn *= 1.0 if "left first" in name else -1.0
        w = slice(i_on, i_on + n)
        out.append({"level": round(abs(lvl), 3), "width": width, "direction": sgn,
                    "wheel": sgn * (d["fine"][w] - d["fine"][i_on]), "van": sgn * (d["vsa_lat"][w] - d["vsa_lat"][i_on]),
                    "cmd": -sgn * (d["cmd_tq"][w] - d["cmd_tq"][i_on]) / STEER_MAX, "motor": sgn * (motor[w] - motor[i_on])})
  return out


def pulse_report(pulses: list[dict]) -> float:
  """Print the held-pulse table; return the smallest pulse whose wheel response clears the no-pulse holds."""
  print(f"\nheld pulses (lane keeping's torque held still; response {PULSE_WINDOW_S:.1f} s after the pulse starts, minus holds with no pulse):")
  print(f"  {'pulse':>6} {'width':>5} {'n':>3} {'command':>8} {'motor':>6} {'wheel deg':>10} {'±':>6} {'van m/s^2':>10}  left/right wheel")
  smallest = np.nan
  for width in sorted({p["width"] for p in pulses}):
    ctrl = [p for p in pulses if p["width"] == width and p["level"] == 0]
    if not ctrl:
      continue
    c_wheel = np.mean([p["wheel"] for p in ctrl], 0)
    c_van = np.mean([p["van"] for p in ctrl], 0)
    noise = np.std([p["wheel"].max() for p in ctrl])
    end = int(min(width + 0.5, PULSE_WINDOW_S) * 100) - 1
    for lvl in sorted({p["level"] for p in pulses if p["width"] == width}):
      g = [p for p in pulses if p["width"] == width and p["level"] == lvl]
      wheel = np.mean([p["wheel"] for p in g], 0) - c_wheel
      se = noise / np.sqrt(len(g)) * np.sqrt(1 + len(g) / len(ctrl))
      sides = []
      for s in (1.0, -1.0):
        gs = [p for p in g if p["direction"] == s]
        sides.append(f"{(np.mean([p['wheel'][end] for p in gs]) - c_wheel[end]) if gs else np.nan:+.3f}")
      cmd = np.mean([p["cmd"][min(int(width * 100) // 2, end)] for p in g])
      mot = np.mean([p["motor"][min(int(width * 100) // 2, end)] for p in g])
      van = np.mean([p["van"][end] for p in g]) - c_van[end]
      print(f"  {lvl:6.3f} {width:5.1f} {len(g):3d} {cmd:+8.3f} {mot:+6.1f} {wheel[end]:+10.3f} {se:6.3f} {van:+10.4f}  {sides[0]}/{sides[1]}")
      if lvl > 0 and wheel[end] > 3 * se and not (smallest <= lvl):
        smallest = lvl
  print("  (command and motor are the change mid-pulse; wheel and van are at 0.5 s after the pulse ends; ± is one standard error)")
  print(f"  smallest pulse that turns the wheel beyond the noise (3 standard errors): {smallest:.3f} of max torque")
  return float(smallest)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("route_dir")
  args = ap.parse_args()
  d, names = load(args.route_dir)
  rs = runs(d)
  print(f"{len(rs)} maneuver runs across {len(names)} maneuvers")
  if not rs:
    print("no lateralManeuverPlan in this drive: was Lateral Maneuver Mode on with LateralManeuverSuite=odyssey?")
    return
  by_name: dict[str, list[np.ndarray]] = defaultdict(list)
  for r in rs:
    by_name[names[int(d["maneuver"][r[0]])]].append(r)
  unknown = [n for n in by_name if schedule(n) is None]
  if unknown:
    print(f"skipping maneuvers no longer in the suite: {', '.join(unknown)}")
  pulses = held_pulses(d, by_name)
  deadband = pulse_report(pulses) if pulses else float("nan")
  if np.isfinite(deadband):
    print("\nfor tools/odyssey_lateral_sim.py:")
    print(f"  --deadband {deadband:.3f}   (smallest held pulse that turns the wheel)")


if __name__ == "__main__":
  main()
